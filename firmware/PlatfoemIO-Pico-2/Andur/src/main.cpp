/*
 * Tauno Erik
 * Hardware:
 *  -Pico 2
 *  -TOF400C/VL53L1X - can handle about 30mm to 1200mm of range distance
 * Started: 29.09.2025
 * Edited:  14.01.2026
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
#include <pico/mutex.h>        // Race Condition Protection
#include "Adafruit_VL53L0X.h"  // Sensor
#include <Adafruit_GFX.h>      // OLED
#include <Adafruit_SSD1306.h>  // OLED
#include "pio_encoder.h"       // https://github.com/gbr1/rp2040-encoder-library
#include <EEPROM.h>

auto_init_mutex(my_mutex);  // Race Condition Protection

/*******************************************************************
 Constants
 *******************************************************************/
constexpr int VERSION = 260107;
// Memory addresses:
constexpr int ADDRESS_MIN = 0; // Rotary Encoder MIN vale
constexpr int ADDRESS_MAX = 4; // A 32-bit integer occupies 4 bytes of memory.
// LED Colors (and pins)
constexpr int GREEN = 2;
constexpr int RED = 3;

// I2C
// SDA - GPIO4
// SCL - GPIO5
//constexpr int SENSOR_GPIO1_PIN = 6;  // Used to indicate that data is ready
constexpr int SENSOR_XSHUT_PIN = 7;  // By default it's pulled high. When the pin is pulled low, the sensor goes into shutdown mode.

constexpr int GREEN_LED_PIN = GREEN;
constexpr int RED_LED_PIN = RED;

// MIN (Left) Rotary Encoder
constexpr int MIN_RE_CLK_PIN = 19; //16;
constexpr int MIN_RE_DT_PIN = 20; //17;
constexpr int MIN_RE_SW_PIN = 21; //18;

// MAX (Right) Rotary Encoder
constexpr int MAX_RE_CLK_PIN = 16; //19;
constexpr int MAX_RE_DT_PIN = 17; //20;
constexpr int MAX_RE_SW_PIN = 18; //21;

// Signal Out
//constexpr int SIGNAL_OUT_1_PIN = 22; // Not used
constexpr int SIGNAL_OUT_2_PIN = 15;

// OLED Screen
constexpr int SCREEN_WIDTH = 128;  // px
constexpr int SCREEN_HEIGHT = 32;  // px
constexpr int OLED_RESET = -1;  // Reset pin # (or -1 if sharing Arduino reset pin)
constexpr uint8_t SCREEN_ADDRESS = 0x3C;  // 0x3D for 128x64, 0x3C for 128x32

// Rotary Encoder
constexpr int ENCODER_MIN_VALUE = 0;
constexpr int ENCODER_MAX_VALUE = 1200; //3000
constexpr int ENCODER_SMALLEST_RANGE = 10;
constexpr int DEBOUNCE_DELAY = 50;

// Counters
constexpr int MAX_ERRORS = 10;
constexpr int CYCLE_MAX_COUNT = 3000; // Reset after

/*******************************************************************
 Global Variables
 *******************************************************************/
volatile uint distance = 0;
volatile int selected_value_min = 0;
volatile int selected_value_max = 0;

volatile int error_count = 0;
volatile int cycle_counter = 0;


// OLED Screen
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Laser sensor
Adafruit_VL53L0X vl53lxx_sensor = Adafruit_VL53L0X();

// Rotary Encoders
PioEncoder min_rotary_encoder(MIN_RE_CLK_PIN);
PioEncoder max_rotary_encoder(MAX_RE_CLK_PIN);


/*******************************************************************
 Functions prototypes
 *******************************************************************/
void i2c_scanner();
void init_LED_pins();
void init_display();
void write_display(long dis, int min, int max);
void init_vl53lxx_sensor();
void recover_vl53lxx_sensor();

void vl53lxx_sensor_OFF();
void vl53lxx_sensor_ON();
void init_ecoders();
void read_buttons();
void read_encoders();
void update_system_outputs(int led_color);

uint32_t read_memory(int address);
int write_memory(int address, uint32_t value);
uint32_t read_memory_min_value();
uint32_t read_memory_max_value();
int write_memory_min_value(uint32_t value);
int write_memory_max_value(uint32_t value);

int process_data(uint local_dis);
void handle_errors(int error);

/*******************************************************************
 SETUP Core 0
 *******************************************************************/
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(921600); // 921600 115200
  //delay(1000);
  //Serial.println("Andur");
  // Do not change! Serial pordi lugemisega tekivad probleemid!!!
  //Wire.setSDA(2); // default 4
  //Wire.setSCL(3); // default 5

  //Wire.setClock(100000); // Standard mode 100 kHz I2C
  // Tekivad probleemid kui kasutada Fast mode
  //Wire.setClock(400000); // Fast mode 400 kHz I2C
  Wire.begin();

  //delay(3000);
  //i2c_scanner();
  
  EEPROM.begin(512);  // Allocate 512 bytes of flash)

  selected_value_min = read_memory_min_value();
  selected_value_max = read_memory_max_value();

  init_vl53lxx_sensor();

  init_display();
}


/*******************************************************************
 SETUP Core 1
 *******************************************************************/
void setup1() {
  init_ecoders();
  init_LED_pins();
  // Init Signal output pin
  pinMode(SIGNAL_OUT_2_PIN,  OUTPUT);
  digitalWrite(SIGNAL_OUT_2_PIN, LOW);
}

/*******************************************************************
 Core 0 loop
 High-speed sensing and visual output
 *******************************************************************/
void loop() {
  static uint local_distace = 0;
  static uint old_local_distace = 0;
  static unsigned long last_display_time = millis();
  static unsigned long last_sensor_success = 0;

  // Sensor
  VL53L0X_RangingMeasurementData_t measure;
  vl53lxx_sensor.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  //Serial.print("measure.RangeStatus ");
  //Serial.print(measure.RangeStatus);
  //Serial.print(" D=");
  int status = measure.RangeStatus;
  if (status == 0) {
    mutex_enter_blocking(&my_mutex);
      distance = measure.RangeMilliMeter; // Global
      // Simple EMA Filter: NewValue = (alpha * CurrentReading) + (1 - alpha) * OldValue
      // Higher alpha (e.g., 0.3) = faster response, lower alpha (e.g., 0.05) = smoother
      local_distace = (uint)(0.2 * distance) + (0.8 * old_local_distace);
      //local_distace = distance;
    mutex_exit(&my_mutex);

    old_local_distace = local_distace;
    last_sensor_success = millis();
    error_count = 0;
    //error_count = process_data(local_distace); // Reset error counter on success

    cycle_counter++;
    Serial.print("Cycle counter: ");
    Serial.print(cycle_counter);
    Serial.print(" : ");

  } else {
    handle_errors(status);
  }

  // Restart time to time
  /*
  if (cycle_counter >= CYCLE_MAX_COUNT) {
    Serial.print("Cycle counter: ");
    Serial.println(cycle_counter);
    recover_vl53lxx_sensor();
    cycle_counter = 0;
  }
    */

  // Watchdog to reset sensor
  if (millis() - last_sensor_success > 2000) {
    recover_vl53lxx_sensor();
    cycle_counter = 0;
  }

  // Propably a error
  if (local_distace > 3000) {
    recover_vl53lxx_sensor();
    cycle_counter = 0;
  }

  // Display data ebery 300 ms
  if ((unsigned long)(millis() - last_display_time) >= 300) {
    Serial.print("* ");
    process_data(local_distace);
    last_display_time = millis();
  }
} // loop() end



/*******************************************************************
 Core 1 loop
 *******************************************************************/
void loop1() {
  //delay(10);
  read_encoders();
  read_buttons();

  //Serial.print("min=");
  //Serial.print(selected_value_min);
  //Serial.print("\tmax=");
  //Serial.print(selected_value_max);
  //Serial.print("\n");
  update_system_outputs(RED); // RED or GREEN LED
}


/******************************************************************
 Scanns connected I2C devices
 and prints addresses.
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


/*******************************************************************
 Initialises Green and Red LED pins.
 *******************************************************************/
void init_LED_pins() {
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
}


/*******************************************************************
 Initialises OLED screen.
 *******************************************************************/
void init_display() {
  Serial.print(F("SSD1306 init: "));
   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   // SSD1306_EXTERNALVCC
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (1) {
      Serial.println("Err 1");
      delay(500);
    }
  }
  Serial.println("SSD1306 OK!");
  
  mutex_enter_blocking(&my_mutex);
    uint local_distace = distance;
  mutex_exit(&my_mutex);

  display.setRotation(2); //rotates text on OLED 1=90 degrees, 2=180 degrees

  write_display(local_distace, selected_value_min, selected_value_max);
}


/*****************************************************************
 @param dis - distance
 @param min - selected_value_min
 @param max - selected_value_max
 *****************************************************************/
void write_display(long dis, int min, int max) {
  display.clearDisplay();
  //display.setRotation(2); //rotates text on OLED 1=90 degrees, 2=180 degrees
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(10,0);

  display.print(dis);
  display.println(F(" mm"));

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(10, 20);

  display.print(F("min:"));
  display.print(min);

  display.setCursor(50, 20);

  display.print(F("  max:"));
  display.print(max);

  display.display();  // Update display
}


/*******************************************************************
 Initialises TOF sensor
 VL53Lxx-v2
 *******************************************************************/
void init_vl53lxx_sensor() {
  Serial.print("Init VL53L0X sensor: ");

  // Shutdown pin
  pinMode(SENSOR_XSHUT_PIN, OUTPUT);
  pinMode(SENSOR_XSHUT_PIN, INPUT_PULLUP);

  while (!vl53lxx_sensor.begin()) {
    Serial.println(F("VL53L0X sensor failed!"));

    vl53lxx_sensor_OFF();
    delay(100);
    vl53lxx_sensor_ON();
    delay(100);
  }

  

  // Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT
  // Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE
  // Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED
  // Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY
  vl53lxx_sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_ACCURACY);
  Serial.println("VL53L0X OK!");
}

/*******************************************************************
 OFF (shutdown mode)
 *******************************************************************/
void vl53lxx_sensor_OFF() {
  digitalWrite(SENSOR_XSHUT_PIN, LOW);
}

/*******************************************************************
 ON
 *******************************************************************/
void vl53lxx_sensor_ON() {
  digitalWrite(SENSOR_XSHUT_PIN, HIGH);
}

/*******************************************************************
 *******************************************************************/
void init_ecoders() {
  min_rotary_encoder.begin();
  min_rotary_encoder.flip();

  max_rotary_encoder.begin();
  max_rotary_encoder.flip();
  // Buttons
  pinMode(MIN_RE_SW_PIN, INPUT_PULLUP);
  pinMode(MAX_RE_SW_PIN, INPUT_PULLUP);

}

/*******************************************************************
 Read Rotary Encoder buttons
 *******************************************************************/
void read_buttons() {
  static int min_btn_state = LOW;
  static int max_btn_state = LOW;
  static int last_min_btn_state = HIGH;
  static int last_max_btn_state = HIGH;

  static unsigned long last_min_time = 0;
  static unsigned long last_max_time = 0;

  // 1. Read
  int min_reading = digitalRead(MIN_RE_SW_PIN);
  int max_reading = digitalRead(MAX_RE_SW_PIN);

  // 2. Is  state changed
  if (min_reading != last_min_btn_state) {
    last_min_time = millis();
  }
  if (max_reading != last_max_btn_state) {
    last_max_time = millis();
  }

  // 3. if time has passed
  if ((millis() - last_min_time) > DEBOUNCE_DELAY) {
    // 4. button state has actually changed
    if (min_reading != min_btn_state) {
      min_btn_state = min_reading;
      // 5.
      if (min_btn_state == LOW) {
        // Serial.println("----MIN");
        // Save MIN value
        write_memory_min_value(selected_value_min);
      }
    }
  }
  last_min_btn_state = min_reading;

  // 3. if time has passed
  if ((millis() - last_max_time) > DEBOUNCE_DELAY) {
    // 4. button state has actually changed
    if (max_reading != max_btn_state) {
      max_btn_state = max_reading;
      // 5.
      if (max_btn_state == LOW) {
        // Serial.println("++++MAX");
        // Save MAX value
        write_memory_max_value(selected_value_max);
      }
    }
  }
  last_max_btn_state = max_reading;
}


/*******************************************************************
 Read Min and Max rotary encoder values
 *******************************************************************/
void read_encoders() {
  static int old_max_encoder_count = 0;
  static int old_min_encoder_count = 0;
  // Rotary Encoder lugemid
  // - kui ei keera on 0
  int new_min_encoder_count = min_rotary_encoder.getCount();
  int new_max_encoder_count = max_rotary_encoder.getCount();

  // MIN
  if (new_min_encoder_count > old_min_encoder_count) {
    mutex_enter_blocking(&my_mutex); // Race Condition Protection
      selected_value_min++;
      if (selected_value_min < ENCODER_MIN_VALUE) {
        selected_value_min = ENCODER_MIN_VALUE;
      } else if (selected_value_min > ENCODER_MAX_VALUE) {
        selected_value_min = ENCODER_MAX_VALUE;
      }
      // et ei oleks suurem, kui max
      if (selected_value_min >= selected_value_max) {
        selected_value_min = selected_value_max - ENCODER_SMALLEST_RANGE;
      }
    mutex_exit(&my_mutex);
  }
  else if (new_min_encoder_count < old_min_encoder_count) {
    mutex_enter_blocking(&my_mutex); // Race Condition Protection
      selected_value_min--;
      if (selected_value_min < ENCODER_MIN_VALUE) {
        selected_value_min = ENCODER_MIN_VALUE;
      } else if (selected_value_min > ENCODER_MAX_VALUE) {
        selected_value_min = ENCODER_MAX_VALUE;
      }
    mutex_exit(&my_mutex);
  }

  // MAX
  if (new_max_encoder_count > old_max_encoder_count) {
    mutex_enter_blocking(&my_mutex); // Race Condition Protection
      selected_value_max++;
      if (selected_value_max < ENCODER_MIN_VALUE) {
        selected_value_max = ENCODER_MIN_VALUE;
      } else if (selected_value_max > ENCODER_MAX_VALUE) {
        selected_value_max = ENCODER_MAX_VALUE;
      }
    mutex_exit(&my_mutex);
  }
  else if (new_max_encoder_count < old_max_encoder_count) {
    mutex_enter_blocking(&my_mutex); // Race Condition Protection
      selected_value_max--;
      if (selected_value_max < ENCODER_MIN_VALUE) {
        selected_value_max = ENCODER_MIN_VALUE;
      } else if (selected_value_max > ENCODER_MAX_VALUE) {
        selected_value_max = ENCODER_MAX_VALUE;
      }
      // Et ei oleks väiksem, kui MIN
      if (selected_value_max <= selected_value_min) {
        selected_value_max = selected_value_min + ENCODER_SMALLEST_RANGE;
      }
    mutex_exit(&my_mutex);
  }

  old_min_encoder_count = new_min_encoder_count;
  old_max_encoder_count = new_max_encoder_count;
}


/*******************************************************************
 Set Signal Output and Staus LED:
 @param color - Select RED or GREEN color if in min/max range.
 *******************************************************************/
void update_system_outputs(int led_color) {
  mutex_enter_blocking(&my_mutex);
   uint local_distace = distance;
  mutex_exit(&my_mutex);

  bool in_range = (local_distace > selected_value_min && local_distace < selected_value_max);

  // Signal Output
  digitalWrite(SIGNAL_OUT_2_PIN, in_range ? HIGH : LOW);

  // LED Logic
  if (led_color == RED) {
    digitalWrite(GREEN_LED_PIN, in_range ? HIGH : LOW);
    digitalWrite(RED_LED_PIN, in_range ? LOW : HIGH);
  } else {
    digitalWrite(GREEN_LED_PIN, in_range ? LOW : HIGH);
    digitalWrite(RED_LED_PIN, in_range ? HIGH : LOW);
  }
}


/*******************************************************************
 @param address  - Memory address to start read
 *******************************************************************/
uint32_t read_memory(int address) {
  uint32_t value = 0;
  EEPROM.get(address, value);
  return value;
}


/*******************************************************************
 @param address  - Memory address to start write
 @param value - A 32-bit integer to write
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


/*******************************************************************
 *******************************************************************/
uint32_t read_memory_min_value() {
  uint32_t value = read_memory(ADDRESS_MIN);
  return value;
}


/*******************************************************************
 *******************************************************************/
uint32_t read_memory_max_value() {
  uint32_t value = read_memory(ADDRESS_MAX);
  return value;
}


/*******************************************************************
 *******************************************************************/
int write_memory_min_value(uint32_t value) {
  int msg = 0;
  // If is a new value then write to memeory
  if (read_memory_min_value() != value) {
    msg = write_memory(ADDRESS_MIN, value);
  }
  return msg;
}


/*******************************************************************
 *******************************************************************/
int write_memory_max_value(uint32_t value) {
  int msg = 0;
  // If is a new value then write to memeory
  if (read_memory_max_value() != value) {
    msg = write_memory(ADDRESS_MAX, value);
  }
  return msg;
}


/*******************************************************************
 Reset vl53lxx sensor
 *******************************************************************/
void recover_vl53lxx_sensor() {
  int delay_val = 70;
  Serial.println("Resetting sensor...");
  
  // 1. Hardware Reset: Pull XSHUT low to shut down the sensor
  vl53lxx_sensor_OFF(); 
  delay(delay_val); // Give it time to fully discharge
  
  // 2. Power back on
  vl53lxx_sensor_ON();
  delay(delay_val); // Wait for the sensor to boot
  
  // 3. Re-initialize the library
  if (!vl53lxx_sensor.begin()) {
    Serial.println("Critical Failure: Sensor could not restart.");
  } else {
    Serial.println("Sensor recovered successfully.");
  }
}


/*******************************************************************
TODO: Ekraanile kirjutamine tuleks teha eraldi protsessiks
      ja teises tuumas jooksutada.
      Et oleks kiirem ja ei segaks muud loogikat.
 *******************************************************************/
int process_data(uint local_dis) {
  static uint old_dis = 0;
  static int old_min = 0;
  static int old_max = 0;

  int local_min = 0;
  int local_max = 0;

  // Simple EMA Filter: NewValue = (alpha * CurrentReading) + (1 - alpha) * OldValue
  // Higher alpha (e.g., 0.3) = faster response, lower alpha (e.g., 0.05) = smoother
  //local_dis = (uint)(0.2 * local_dis) + (0.8 * old_dis);
  Serial.println(local_dis);

  mutex_enter_blocking(&my_mutex);
    local_min = selected_value_min;
  mutex_exit(&my_mutex);

  mutex_enter_blocking(&my_mutex);
    local_max = selected_value_max;
  mutex_exit(&my_mutex);

  // Update display only when there is new data
  if (local_dis != old_dis || local_min != old_min || local_max != old_max) {
    write_display(local_dis, local_min, local_max);
    old_dis = local_dis;
    old_min = local_min;
    old_max = local_max;
  }

  return 0;
}


/*******************************************************************
 *******************************************************************/
void handle_errors(int error) {
  switch (error) {
    case 1:
      error_count++;
      Serial.print("Sigma fail ");
      Serial.println(error_count);
      break;
    case 2:
      error_count++;
      Serial.print("Signal fail ");
      Serial.println(error_count);
      break;
    case 3:
      error_count++;
      Serial.print("Range wrap ");
      Serial.println(error_count);
      break;
    case 4:
      error_count++;
      Serial.print("Out of range ");
      Serial.println(error_count);
      break;
    default:
      error_count++;
      Serial.print("Error count: ");
      Serial.println(error_count);
      break;
  }

  if (error_count >= MAX_ERRORS) {
    Serial.print("Error count: ");
    Serial.println(error_count);
    recover_vl53lxx_sensor();
    error_count = 0; // Reset counter after attempting recovery
  }
}
