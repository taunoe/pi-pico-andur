/*
 * Tauno Erik
 * Hardware:
 *  -Pico 2
 *  -TOF400C/VL53L1X
 * Started: 29.09.2025
 * Edited:  02.01.2026
 *
 * Links:
 * - https://arduino-pico.readthedocs.io/en/latest/platformio.html
 * - https://randomnerdtutorials.com/raspberry-pi-pico-i2c-scanner-arduino/
 * - https://github.com/pololu/vl53l1x-arduino/
 * - https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library
 * - https://github.com/adafruit/Adafruit_VL53L0X/tree/master
 * - https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/arduino-code
 * - https://github.com/gbr1/rp2040-encoder-library
 *
 * I2C0: SDA GPIOs -> GPIO0, GPIO4, GPIO8, GPIO12, GPIO16, GPIL20
 * I2C0: SCL GPIOs -> GPIO1, GPIO5, GPIO9, GPIO13, GPIO17, GPIO21
 *
 * I2C1: SDA GPIOs -> GPIO2, GPIO6, GPIO10, GPIO14, GPIO18, GPIO26
 * I2C1: SCL GPIOs -> GPIO3, GPIO7, GPIO11, GPIO15, GPIO19, GPIO27
 *
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_VL53L0X.h"  // Sensor
#include <Adafruit_GFX.h>      // OLED
#include <Adafruit_SSD1306.h>  // OLED
#include "pio_encoder.h"       // https://github.com/gbr1/rp2040-encoder-library
#include <EEPROM.h>


/********************************
  Pins
*********************************/
// I2C
// SDA - GPIO4
// SCL - GPIO5
#define SENSOR_GPIO1_PIN    6  // Used by the sensor to indicate that data is ready
#define SENSOR_XSHUT_PIN    7  // By default it's pulled high. When the pin is pulled low, the sensor goes into shutdown mode.

#define MIN_VAL             0
#define MAX_VAL             1
#define GREEN               2
#define RED                 3
#define GREEN_LED_PIN   GREEN
#define RED_LED_PIN       RED

// MIN (Left) Rotary Encoder
#define MIN_RE_CLK_PIN     16
#define MIN_RE_DT_PIN      17
#define MIN_RE_SW_PIN      18
#define ADDRESS_MIN         0

// MAX (Right) Rotary Encoder
#define MAX_RE_CLK_PIN     19
#define MAX_RE_DT_PIN      20
#define MAX_RE_SW_PIN      21
#define ADDRESS_MAX         4 // A 32-bit integer occupies 4 bytes of memory.

// Signal Out
#define SIGNAL_OUT_1_PIN   22
#define SIGNAL_OUT_2_PIN   15


/********************************
  OLED Screen
*********************************/
#define SCREEN_WIDTH      128  // px
#define SCREEN_HEIGHT      32  // px
#define OLED_RESET         -1  // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS   0x3C  // 0x3D for 128x64, 0x3C for 128x32

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
int selected_value_min = 10;
int selected_value_max = 100;

#define ENCODER_MIN_VALUE         0
#define ENCODER_MAX_VALUE      1000 //3000
#define ENCODER_SMALLEST_RANGE    5


/********************************
  Rotary Encoders
*********************************/
// MIN Rotary Encoder
PioEncoder min_rotary_encoder(MIN_RE_CLK_PIN);
// MAX Rotary Encoder
PioEncoder max_rotary_encoder(MAX_RE_CLK_PIN);


/********************************
  Functions prototypes
*********************************/
void i2c_scanner();
void init_LED_pins();
void init_display();
void init_vl53lxx_sensor();

void vl53lxx_sensor_OFF();
void vl53lxx_sensor_ON();
void init_ecoders();
void read_buttons();
void read_encoders();
void set_status_LED(int color);

uint32_t read_memory(int address);
int write_memory(int address, uint32_t value);
uint32_t read_memory_min_value();
uint32_t read_memory_max_value();
int write_memory_min_value(uint32_t value);
int write_memory_max_value(uint32_t value);

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

  //delay(3000);
  //i2c_scanner();

  init_LED_pins();
  init_display();
  init_vl53lxx_sensor();

  // Rotary Encoders
  init_ecoders();

  // Initialize EEPROM (allocate 512 bytes of flash)
  EEPROM.begin(512);
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

  //Serial.print("measure.RangeStatus ");
  //Serial.print(measure.RangeStatus);
  //Serial.print(" D=");

  if (measure.RangeStatus != 4) {
    // Distance
    distance = measure.RangeMilliMeter;
    //Serial.print(distance);

    // Calculate average distamce
    distance_history[dis_history_i] = distance;

    if (++dis_history_i == DISTANCE_HISTORY_SIZE) {
      dis_history_i = 0;
    }

    for (int x = 0; x < DISTANCE_HISTORY_SIZE; x++) {
      avg_distance += distance_history[x];
    }

    avg_distance /= DISTANCE_HISTORY_SIZE;
    //Serial.print("\tavg=");
    //Serial.print(avg_distance);
    //Serial.print(" ");

    // Display on screen
    display.clearDisplay();

    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10,0);
    display.print(distance);  //or avg_distance
    display.println(F(" mm"));

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(10,20);
    display.print(F("min:"));
    display.print(selected_value_min);

    display.setCursor(50,20);
    display.print(F("  max:"));
    display.print(selected_value_max);

    display.display();  // Update display
  }

  long end_time = millis();

  //vl53lxx_sensor_OFF();
  //delay(10);
  //vl53lxx_sensor_ON();

  //Serial.println();
  delay(90);
}


/********************************
  Core 1 loop
*********************************/
void loop1() {
  read_encoders();
  read_buttons();

  //Serial.print("min=");
  //Serial.print(selected_value_min);
  //Serial.print("\tmax=");
  //Serial.print(selected_value_max);
  //Serial.print("\n");

  set_status_LED(RED); // RED or GREEN
}


/******************************************************************
 * Scanns connected I2C devices
 * and prints addresses.
 ******************************************************************/
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
 * Initialises Green and Red LED pins.
 *********************************/
void init_LED_pins() {
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}


/********************************
 * Initialises OLED screen.
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
 * Initialises TOF sensor
 * VL53Lxx-v2
 ********************************/
void init_vl53lxx_sensor() {
  Serial.println("Init VL53L0X sensor");

  // Shutdown pin
  pinMode(SENSOR_XSHUT_PIN, OUTPUT);

  if (!vl53lxx_sensor.begin()) {
    Serial.println(F("VL53L0X sensor failed!"));

    while (1) {
      Serial.println("Err 2");
      delay(500);
    }
  }
}

/********************************
 * OFF (shutdown mode)
 ********************************/
void vl53lxx_sensor_OFF() {
  digitalWrite(SENSOR_XSHUT_PIN, HIGH);
}

/********************************
 * ON
 ********************************/
void vl53lxx_sensor_ON() {
  digitalWrite(SENSOR_XSHUT_PIN, HIGH);
}

void init_ecoders() {
  min_rotary_encoder.begin();
  min_rotary_encoder.flip();

  max_rotary_encoder.begin();
  max_rotary_encoder.flip();
  // Buttons
  pinMode(MIN_RE_SW_PIN, INPUT_PULLUP);
  pinMode(MAX_RE_SW_PIN, INPUT_PULLUP);

}

void read_buttons() {
  static int last_min_btn_state = LOW;
  static int last_max_btn_state = LOW;
  // Read
  int min_btn_state = digitalRead(MIN_RE_SW_PIN);
  int max_btn_state = digitalRead(MAX_RE_SW_PIN);

  if (min_btn_state != last_min_btn_state) {
    if (min_btn_state == HIGH) {
      Serial.println("----MIN BTN");
      last_min_btn_state = min_btn_state;
    }
  }
  
  
  if (max_btn_state != last_max_btn_state) {
    if (min_btn_state == HIGH) {
      Serial.println("----MAX BTN");
      last_max_btn_state = max_btn_state;
    }
  }
  

  delay(10);
}

/****************************************************
 * Read Min and Max rotary encoder values
 ****************************************************/
void read_encoders() {
  static int old_max_encoder_count = 0;
  static int old_min_encoder_count = 0;
  // Rotary Encoder lugemid
  // - kui ei keera on 0
  int new_min_encoder_count = min_rotary_encoder.getCount();
  int new_max_encoder_count = max_rotary_encoder.getCount();

  // MIN
  if (new_min_encoder_count > old_min_encoder_count) {
    selected_value_min++;
  }
  else if (new_min_encoder_count < old_min_encoder_count) {
    selected_value_min--;
  }

  // MAX
  if (new_max_encoder_count > old_max_encoder_count) {
    selected_value_max++;
  }
  else if (new_max_encoder_count < old_max_encoder_count) {
    selected_value_max--;
  }

  // clamp value
  if (selected_value_max < ENCODER_MIN_VALUE) {
    selected_value_max = ENCODER_MIN_VALUE;
  } else if (selected_value_max > ENCODER_MAX_VALUE) {
    selected_value_max = ENCODER_MAX_VALUE;
  }

  old_min_encoder_count = new_min_encoder_count;
  old_max_encoder_count = new_max_encoder_count;
}


/*******************************************************************
 * Set staus led:
 *   @param color - RED or GREEN color if in min/max range.
 *******************************************************************/
void set_status_LED(int color) {
  // If not in min-max range
  if (distance < selected_value_min || distance > selected_value_max) {
    if (color == RED) {
      // Red LED ON
      digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
    }
    else {
      // Green LED ON
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, LOW);
    }
    
  }
  else {
     if (color == RED) {
      // Green LED ON
      digitalWrite(GREEN_LED_PIN, HIGH);
      digitalWrite(RED_LED_PIN, LOW);
     }
     else {
      // Red LED ON
       digitalWrite(GREEN_LED_PIN, LOW);
      digitalWrite(RED_LED_PIN, HIGH);
     }
  }
}

/*******************************************************************
 * @param address  - Memory address to start read
 *******************************************************************/
uint32_t read_memory(int address) {
  uint32_t value = 0;
  EEPROM.get(address, value);
  return value;
}

/*******************************************************************
 * @param address  - Memory address to start write
 * @param value - A 32-bit integer to write
 *******************************************************************/
int write_memory(int address, uint32_t value) {
  EEPROM.put(address, value);
  // Must commit to save to physical Flash!
  if (EEPROM.commit()) {
    return 0; // OK
  } else {
    return 1; // Error
  }
}

uint32_t read_memory_min_value() {
  uint32_t value = read_memory(ADDRESS_MIN);
  return value;
}

uint32_t read_memory_max_value() {
  uint32_t value = read_memory(ADDRESS_MAX);
  return value;
}

int write_memory_min_value(uint32_t value) {
  int msg = write_memory(ADDRESS_MIN, value);
  return msg;
}

int write_memory_max_value(uint32_t value){
  int msg = write_memory(ADDRESS_MAX, value);
  return msg;
}
