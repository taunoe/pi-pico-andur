/*
 * Tauno Erik
 * Hardware:
 *  -Pico 2
 *  -TOF400C/VL53L1X
 * Started: 29.09.2025
 * Edited:  11.12.2025
 * 
 * Links:
 * - https://arduino-pico.readthedocs.io/en/latest/platformio.html
 * - https://randomnerdtutorials.com/raspberry-pi-pico-i2c-scanner-arduino/
 * - https://github.com/pololu/vl53l1x-arduino/
 * - https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library
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
//#include "SparkFun_VL53L1X.h"
#include "Adafruit_VL53L0X.h"  // https://github.com/adafruit/Adafruit_VL53L0X/tree/master
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// https://www.upesy.com/blogs/tutorials/rotary-encoder-raspberry-pi-pico-with-arduino-code#
#include "hardware/pio.h"
#include "quadrature.pio.h"  // https://github.com/jamon/pi-pico-pio-quadrature-encoder/tree/main


#define LONG  0
#define SHORT 1

/* Pins */
// Optional interrupt and shutdown pins for sensor.
// #define SHUTDOWN_PIN 2 // ?
// #define INTERRUPT_PIN 3 // ?
#define STATUS_GREEN_LED   14  //
#define STATUS_RED_LED     15  //
#define SELECTED_GREEN_LED 16  // Selected distance
#define SELECTED_RED_LED   17  // Selected distance

#define MIN_RE_CLK_PIN     18
#define MIN_RE_DT_PIN      19
#define MAX_RE_CLK_PIN     20
#define MAX_RE_DT_PIN      21



/* OLED Screen */
#define SCREEN_WIDTH      128  // OLED display width, in pixels
#define SCREEN_HEIGHT      32  // OLED display height, in pixels
#define OLED_RESET         -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS   0x3C  // < See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


/* Laser sensor */
// adafruit
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
/*
// Sparcfun code
SFEVL53L1X distance_sensor;
//Uncomment the following line to use the optional shutdown and interrupt pins.
//SFEVL53L1X distance_sensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
//Store distance readings to get rolling average
#define DISTANCE_HISTORY_SIZE 3
int distance_history[DISTANCE_HISTORY_SIZE] = {0};
uint8_t dis_history_i = 0;
*/


/* Global Variables */
long avg_distance = 0;
uint distance = 0;
uint selected_value_min = 10;
uint selected_value_max = 100;



#define ENCODER_MIN_VALUE         0
#define ENCODER_MAX_VALUE      4000
#define ENCODER_SMALLEST_RANGE    5
/* Rotary Encoder MAX */
int32_t current_encoder_value_max = selected_value_max;
uint32_t last_pio_count_max = 0;
PIO re_pio_max = pio0;
uint re_offset_max;
uint re_sm_max;

/* Rotary Encoder MIN */
int32_t current_encoder_value_min = selected_value_min;
uint32_t last_pio_count_min = 0;
PIO re_pio_min = pio0;
uint re_offset_min;
uint re_sm_min;


/* Functions */
void i2c_scanner();
void init_LED_pins();
void init_display();
void init_sensor();
void set_sensor_mode(int mode);

void setup() {
  Serial.begin(115200); // 921600

  delay(3000);
  i2c_scanner();

  // Do not change! Serial pordi lugemisega tekivad probleemid!!!
  //Wire.setSDA(2); // default 4
  //Wire.setSCL(3); // default 5
  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  init_LED_pins();
  init_display();
  init_sensor();
  set_sensor_mode(LONG);


  // Rotary encoder 1 MAX
  re_offset_max = pio_add_program(re_pio_max, &quadratureA_program);
  re_sm_max = pio_claim_unused_sm(re_pio_max, true);
  quadratureA_program_init(re_pio_max, re_sm_max, re_offset_max, MAX_RE_CLK_PIN, MAX_RE_DT_PIN);

  // Rotary encoder 2 MIN
  re_offset_min = pio_add_program(re_pio_min, &quadratureA_program);
  re_sm_min = pio_claim_unused_sm(re_pio_min, true);
  quadratureA_program_init(re_pio_min, re_sm_min, re_offset_min, MIN_RE_CLK_PIN, MIN_RE_DT_PIN);
}

// Core 0
void loop() {
  long start_time = millis();

  /* Sensor */
  distance_sensor.startRanging();  // Write configuration bytes to initiate measurement

  while (!distance_sensor.checkForDataReady()) {
    delay(1);
  }
  distance = distance_sensor.getDistance();  // Get the result of the measurement from the sensor
  distance_sensor.clearInterrupt();
  distance_sensor.stopRanging();
  long end_time = millis();

  //Serial.print(distance);  // Kaugus mm

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

  int signalRate = distance_sensor.getSignalRate();
  Serial.print("\tSignal rate: ");
  Serial.print(signalRate);

  byte range_status = distance_sensor.getRangeStatus();
  Serial.print("\tRange Status: ");

  //Make it human readable
  switch (range_status)
  {
  case 0:
    Serial.print("Good");
    break;
  case 1:
    Serial.print("Sigma fail");
    break;
  case 2:
    Serial.print("Signal fail");
    break;
  case 7:
    Serial.print("Wrapped target fail");
    break;
  default:
    Serial.print("Unknown: ");
    Serial.print(range_status);
    break;
  }

  Serial.print("\tHz: ");
  Serial.print(1000.0 / (float)(end_time - start_time), 2);

  Serial.println();


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


// Core 1
void loop1() {
  if (distance < selected_value_min || distance > selected_value_max) {
    // Red LED ON
    digitalWrite(SELECTED_GREEN_LED, HIGH);
    digitalWrite(SELECTED_RED_LED, LOW);
  }
  else {
    // Green LED ON
    digitalWrite(SELECTED_GREEN_LED, LOW);
    digitalWrite(SELECTED_RED_LED, HIGH);
  }

}


/*
 * Scanns connected I2C devices
 * and prints addresses
 */
void i2c_scanner() {
  byte error, address;
  int nDevices = 0;
  Serial.println("Scanning...");

  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    delay(10);
    error = Wire.endTransmission();
    delay(10);
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

/*
*/
void init_LED_pins() {
  pinMode(SELECTED_GREEN_LED, OUTPUT);
  pinMode(SELECTED_RED_LED, OUTPUT);

  digitalWrite(SELECTED_RED_LED, LOW);
  digitalWrite(SELECTED_GREEN_LED, LOW);
}

/*
*/
void init_display() {
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   // SSD1306_EXTERNALVCC
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1) {
      Serial.println("Err 1");
      delay(100);
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

/*
*/
void init_sensor() {
  /*
  if (distance_sensor.begin() != 0) {
    Serial.println("Sensor connection failed.");
    while (1) {
      Serial.println("Err 2");
      delay(500);
    }
  }
    */
    Serial.println("Adafruit VL53L0X test");
    if (!lox.begin()) {
      Serial.println(F("Failed to boot VL53L0X"));
      while(1);
    }
/*
  while (distance_sensor.begin() != 0) {
    Serial.println("Err 2");
    Serial.println("Sensor connection failed.");
    delay(500);
  }
  Serial.println("Sensor works!");
  */
}

/*
*/
void set_sensor_mode(int mode) {
  if (mode == LONG) {
    distance_sensor.setDistanceModeLong();
  }
  else if (mode == SHORT) {
    distance_sensor.setDistanceModeShort();
  }
}
