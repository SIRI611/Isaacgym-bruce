/*
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "September 6, 2023"
__version__   = "0.0.3"
__status__    = "Production"
*/
 
#define calibration false  // Set to True to calibrate IMU

#include "Servo.h"
#include "math.h"
#include "ISM330DHCX.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"

#include "Adafruit_NeoPixel.h"
#ifdef __AVR__
  #include <avr/power.h>   // Required for 16 MHz Adafruit Trinket
#endif

/* Serial Communication
IMU Control Table
==STAT=====================================
  NA      0x00
  a_x     0x01  // compensated acceleration, m/s
  a_y     0x02
  a_z     0x03
  omega_x 0x04  // compensated omega, rad/s
  omega_y 0x05
  omega_z 0x06
==CFG=====================================
  dt          0x00  // loop time
  a_bias_x    0x01  // acceleration bias in m/s
  a_bias_y    0x02
  a_bias_z    0x03
  gyro_bias_x 0x04  // gyro bias in rad/s
  gyro_bias_y 0x05
  gyro_bias_z 0x06 */

typedef union {
  float floatingPoint;
  uint8_t binary[4];
} float32;  // content of binary[0] is the lowest LSByte

typedef union {
  short shortInt;
  uint8_t binary[2];
} int16;  // content of binary[0] is the lowest LSByte

float32 STAT[7] = {0};  // STAT data space
float32  CFG[7] = {0};  //  CFG data space

// Pointers for easier programming
// ==STAT=========================

// ==CFG==========================
float32 *dt          = &CFG[0];  // loop time in us
float32 *a_bias_x    = &CFG[1];  // acceleration bias
float32 *a_bias_y    = &CFG[2];
float32 *a_bias_z    = &CFG[3];
float32 *gyro_bias_x = &CFG[4];  // gyro bias
float32 *gyro_bias_y = &CFG[5];
float32 *gyro_bias_z = &CFG[6];

uint8_t   error = 0b10000000;  // Error LSB: overtime, xx,xx,xx...
uint8_t contact = 0b10000000;  // Contact status, MSB always 1 and the last 4 bits from MSB: left foot toe heel, right foot toe heal

// Pinout
static const uint SDA_PIN = 12;
static const uint SCL_PIN = 13;

static const uint  LTOE_PIN = 9;
static const uint LHEEL_PIN = 8;
static const uint  RTOE_PIN = 10;
static const uint RHEEL_PIN = 11;

static const uint FAN_PIN  = 6;
static const uint PUMP_PIN = 7;

static const uint NEO_PIN = 16;

// Ports
i2c_inst_t *i2c = i2c0;

// Other constants
static const float us2s = 0.000001;    // microsecond to second
static const float dt_default = 500;   // default dt [us] -> 2 kHz

// Global variables
unsigned long t_final = 0;             // loop end time in microsecond
float loop_time = 0;                   // loop duration
float gyro_scale = 1;                  // range is in milli-dps per bit!
float accel_scale = 1;                 // range is in milli-g per bit!
uint8_t IMU_timestamp = 0b00000001;    // IMU internal timestamp

// IMU moving average
static const uint Q_LEN = 10;          // Moving average queue length
float Q_X[Q_LEN+1] = {0};              // Global queue space for a_x, last element stores queue average
float Q_Y[Q_LEN+1] = {0};              // Global queue space for a_y, last element stores queue average
float Q_Z[Q_LEN+1] = {0};              // Global queue space for a_z, last element stores queue average
float W_X[Q_LEN+1] = {0};              // Global queue space for w_x, last element stores queue average
float W_Y[Q_LEN+1] = {0};              // Global queue space for w_y, last element stores queue average
float W_Z[Q_LEN+1] = {0};              // Global queue space for w_z, last element stores queue average

// Temperature and mode related
float pico_mode = 1.0;                 // idle(0); run(1); reset(2)
float temperature = 20.0;              // temperature data (update if reading is reasonable)
float temp_reading = 20.0;             // temperature reading
float last_update_time = 0;
static const uint pkg_len = 6;         // temperature packet length
unsigned char pkg[pkg_len];            // temperature packet
static const float temp_min = 0.0;     // temperature minimum
static const float temp_max = 80.0;    // temperature maximum
static const float update_freq = 1;    // update temperature and mode at 1 Hz
static const float update_dt = 1 / update_freq / us2s;

// Create servo object to control the pump
Servo PUMP;
static const float pump_rest_temp = 65.0;    // pump resting temperature
static const float pump_work_temp = 70.0;    // pump working temperature
static const float pump_speed = 70.0;        // 0-180
static const float fan_speed  = 99.0;        // 0-100
bool pump_working = false;

// Create pixel object to control LED
static const uint NUM_OF_PIXELS = 1;
Adafruit_NeoPixel pixels(NUM_OF_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

/* Function Declarations */

int reg_write(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);

int  reg_read(i2c_inst_t *i2c, const uint addr, const uint8_t reg, uint8_t *buf, const uint8_t nbytes);

void read_IMU();

void check_IMU();

void setup_IMU();

void read_contact();

void dump();

void LED_status();

float moving_average(float *q, float new_data);

void update_temperature_and_mode();

float byte2float(char byte0, char byte1, char byte2, char byte3);

void update_pump_and_fan_signal();

float my_map(float input, float input_min, float input_max, float output_min, float output_max);

void get_IMU_bias();

void pico_mode_setting();

/* Function Definitions */

// Write 1 byte to the specified register
int reg_write(i2c_inst_t *i2c,
              const uint addr,
              const uint8_t reg,
              uint8_t *buf,
              const uint8_t nbytes) {

  int num_bytes_read = 0;
  uint8_t msg[nbytes+1];

  // Check to make sure caller is sending 1 or more bytes
  if (nbytes < 1) {
      return 0;
  }

  // Append register address to front of data packet
  msg[0] = reg;
  for (int i = 0; i < nbytes; i++) {
      msg[i+1] = buf[i];
  }

  // Write data to register(s) over I2C
  i2c_write_blocking(i2c, addr, msg, (nbytes+1), false);

  return num_bytes_read;
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive registers.
int reg_read(i2c_inst_t *i2c,
             const uint addr,
             const uint8_t reg,
             uint8_t *buf,
             const uint8_t nbytes) {

  int num_bytes_read = 0;

  // Check to make sure caller is asking for 1 or more bytes
  if (nbytes < 1) {
      return 0;
  }

  // Read data from register(s) over I2C
  i2c_write_blocking(i2c, addr, &reg, 1, true);
  num_bytes_read = i2c_read_blocking(i2c, addr, buf, nbytes, false);

  return num_bytes_read;
}

// Calculate moving average
float moving_average(float *q, float new_data) {
  q[Q_LEN] -= q[0] / Q_LEN;  // Remove the portion of the first value in the queue from average
  for (int i = 0; i< Q_LEN-1; i++) {
    q[i] = q[i+1]; // Shift queue forward
  }
  q[Q_LEN-1] = new_data;            // Add new data into queue
  q[Q_LEN]  += q[Q_LEN-1] / Q_LEN;  // Add the portion of the last value in the queue into average
  return q[Q_LEN];
}

// Read and filter data from IMU, then populate STAT date space
void read_IMU() {
  // Buffer to store raw reads
  uint8_t data[12];

  // Declare data processing buffer
  int16_t acc_x, acc_y, acc_z;
  int16_t gyro_x, gyro_y, gyro_z;
  float acc_x_f, acc_y_f, acc_z_f;
  float gyro_x_f, gyro_y_f, gyro_z_f;

  // Read X, Y, and Z acc and gyro values from registers (16 bits each, 12 bytes in total)
  reg_read(i2c, ISM330DHCX_ADDR, REG_X_L_G, data, 12);

  // Convert 2 bytes (little-endian) into 16-bit integer (signed)
  gyro_x = (int16_t)((data[1]  << 8) | data[0]);
  gyro_y = (int16_t)((data[3]  << 8) | data[2]);
  gyro_z = (int16_t)((data[5]  << 8) | data[4]);
  acc_x  = (int16_t)((data[7]  << 8) | data[6]);
  acc_y  = (int16_t)((data[9]  << 8) | data[8]);
  acc_z  = (int16_t)((data[11] << 8) | data[10]);

  // Convert measurements to [m/s^2] and [rad/s]
  gyro_x_f = gyro_x * gyro_scale;
  gyro_y_f = gyro_y * gyro_scale;
  gyro_z_f = gyro_z * gyro_scale;
  acc_x_f  = acc_x * accel_scale;
  acc_y_f  = acc_y * accel_scale;
  acc_z_f  = acc_z * accel_scale;

  // Unbias
  float a[] = {acc_x_f - (*a_bias_x).floatingPoint,
               acc_y_f - (*a_bias_y).floatingPoint,
               acc_z_f - (*a_bias_z).floatingPoint};
  float omega[] = {gyro_x_f - (*gyro_bias_x).floatingPoint,
                   gyro_y_f - (*gyro_bias_y).floatingPoint,
                   gyro_z_f - (*gyro_bias_z).floatingPoint};

  // Filter data with moving average
  a[0] = moving_average(Q_X, a[0]);
  a[1] = moving_average(Q_Y, a[1]);
  a[2] = moving_average(Q_Z, a[2]);
  omega[0] = moving_average(W_X, omega[0]);
  omega[1] = moving_average(W_Y, omega[1]);
  omega[2] = moving_average(W_Z, omega[2]);

  // Write data to STAT space
  for (int i = 0; i < 3; i++) {
    // compensated raw acceleration
    STAT[1+i].floatingPoint = a[i];

    // compensated raw omega
    STAT[4+i].floatingPoint = omega[i];
  }

  // Check IMU status
  check_IMU();
}

void setup_IMU() {
  // Enable IMU timestamp counter
  uint8_t CTRL10_C = 0b00100000;
  reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL10_C, &CTRL10_C, 1);

  // Set Accelerometer Control register for 1.66 kHz, 4g, data from LPF1 (0b1000 10 0 0)
  uint8_t CTRL1_XL = 0b10001000;
  reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL1_XL, &CTRL1_XL, 1);

  // Set Gyro Control register for 1.66 kHz, 500DPS (0b1000 01 0 0)
  uint8_t CTRL2_G = 0b10000100;
  reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL2_G, &CTRL2_G, 1);

  // Decide scale factors from range selections
  ism330dhcx_gyro_range_t gyro_range = ISM330DHCX_GYRO_RANGE_500_DPS;
  switch (gyro_range) {
    case ISM330DHCX_GYRO_RANGE_4000_DPS: gyro_scale = 140.0; break;
    case ISM330DHCX_GYRO_RANGE_2000_DPS: gyro_scale = 70.00; break;
    case ISM330DHCX_GYRO_RANGE_1000_DPS: gyro_scale = 35.00; break;
    case ISM330DHCX_GYRO_RANGE_500_DPS:  gyro_scale = 17.50; break;
    case ISM330DHCX_GYRO_RANGE_250_DPS:  gyro_scale = 8.750; break;
    case ISM330DHCX_GYRO_RANGE_125_DPS:  gyro_scale = 4.375; break;
    default: break;
  }
  ism330dhcx_accel_range_t accel_range = ISM330DHCX_ACCEL_RANGE_4_G;
  switch (accel_range) {
    case ISM330DHCX_ACCEL_RANGE_16_G: accel_scale = 0.48828125000; break;
    case ISM330DHCX_ACCEL_RANGE_8_G:  accel_scale = 0.24414062500; break;
    case ISM330DHCX_ACCEL_RANGE_4_G:  accel_scale = 0.12207031250; break;
    case ISM330DHCX_ACCEL_RANGE_2_G:  accel_scale = 0.06103515625; break;
    default: break;
  }
  gyro_scale  *= SENSORS_DPS_TO_RADS / 1000.0;
  accel_scale *= SENSORS_GRAVITY_STANDARD / 1000.0;
}

// Check IMU is working normally
void check_IMU() {
  uint8_t data[1];
  reg_read(i2c, ISM330DHCX_ADDR, REG_TIMESTAMP0, data, 1);
  if (data[0] == IMU_timestamp) {  // 2 identical timestamp (impossible!)
    // so reset IMU
    uint8_t CTRL3_C = 0b00000101;
    reg_write(i2c, ISM330DHCX_ADDR, REG_CTRL3_C, &CTRL3_C, 1);

    setup_IMU();

    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(150, 0, 150, 150));  // cyan
    pixels.show();
  }
  IMU_timestamp = data[0];
}

// Get Foot Contact Status and update 'contact'
void read_contact() {
  // contact = 0b00000000 (contact status, the last 4 bits: left foot toe heel, right foot toe heal)
  contact = gpio_get(LTOE_PIN)<<3 | gpio_get(LHEEL_PIN)<<2 | gpio_get(RTOE_PIN)<<1 | gpio_get(RHEEL_PIN);  
}

// Dump the data to serial port
void dump() {
  /* Packet: [0xFF 0xFF packetLen error
              ACCEL_X ACCEL_Y ACCEL_Z OMEGA_X OMEGA_Y OMEGA_Z
              contact 0x0F] */
  uint8_t rtnPkt[30] = {};
  rtnPkt[0]  = 0xFF;
  rtnPkt[1]  = 0xFF;
  rtnPkt[2]  = 27;
  rtnPkt[3]  = error;
  rtnPkt[28] = contact;
  rtnPkt[29] = 0x0F;
  for (int i = 1; i < 7; i++) {
    // build packet
    rtnPkt[i*4]   = STAT[i].binary[0];
    rtnPkt[i*4+1] = STAT[i].binary[1];
    rtnPkt[i*4+2] = STAT[i].binary[2];
    rtnPkt[i*4+3] = STAT[i].binary[3];
  }
  Serial.write(rtnPkt, 30);
}

// Show error and contact status via LED
void LED_status() {
  uint8_t LF = 3 - (uint8_t(contact&0b1100)>>2);
  uint8_t RF = 3 - uint8_t(contact&0b0011);
  pixels.setPixelColor(0, pixels.Color((error-128)*100, LF*50, RF*50, 150));
  pixels.show();
}

// Update the temperature and Pico mode
void update_temperature_and_mode() {
  if (Serial.available()) {
    if (Serial.available() == pkg_len) {
      Serial.readBytesUntil(0xFE, pkg, pkg_len);
      if (pkg[0] == 0xFF) {  // check the first byte
        temp_reading = byte2float(0, 0, pkg[1], pkg[2]);
        pico_mode    = byte2float(0, 0, pkg[3], pkg[4]);
        if (temp_reading>=temp_min && temp_reading<=temp_max) temperature = temp_reading;  // check the reading
      }
    }
    while(Serial.available()) Serial.read();  // flush the input buffer anyway
  }
}

// Convert 4-byte to float32
float byte2float(char byte0, char byte1, char byte2, char byte3) {
  float32 t;
  t.binary[0] = byte0;
  t.binary[1] = byte1;
  t.binary[2] = byte2;
  t.binary[3] = byte3;
  return t.floatingPoint;
}

// Update pump and fan signal
void update_pump_and_fan_signal() {
  if (pump_working and temperature < pump_rest_temp) {
    pump_working = false;
    PUMP.writeMicroseconds(900);
    pwm_set_gpio_level(FAN_PIN, 0);
  } else if (not pump_working and temperature > pump_work_temp) {
    pump_working = true;
    PUMP.write(pump_speed);
    pwm_set_gpio_level(FAN_PIN, fan_speed);
  }
}

// Customized linear map function
float my_map(float input, float input_min, float input_max, float output_min, float output_max) {
  if (input < input_min) {
    return output_min;
  } else if (input > input_max) {
    return output_max;
  } else {
    return output_min + (input - input_min) / (input_max - input_min) * (output_max - output_min);
  }
}

// Get IMU bias info
void get_IMU_bias() {
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(150, 0, 150, 150));  // cyan
  pixels.show();

  sleep_ms(1000);
    
  Serial.println("IMU in calibration ...");
  float ba_x, ba_y, ba_z, bw_x, bw_y, bw_z;
  int count = 10000;
  for (int i = 0; i < count; i++) {
    read_IMU();
    ba_x += STAT[1].floatingPoint / count;
    ba_y += STAT[2].floatingPoint / count;
    ba_z += STAT[3].floatingPoint / count;
    bw_x += STAT[4].floatingPoint / count;
    bw_y += STAT[5].floatingPoint / count;
    bw_z += STAT[6].floatingPoint / count;
    sleep_ms(1);
  }
  Serial.println("IMU accel biases:");
  Serial.println(ba_x, 10);
  Serial.println(ba_y, 10);
  Serial.println(ba_z - 9.81, 10);
  Serial.println();
  Serial.println("IMU gyro biases:");
  Serial.println(bw_x, 10);
  Serial.println(bw_y, 10);
  Serial.println(bw_z, 10);
  while (true);  // Stall device
}

void pico_mode_setting() {
  // Idle Pico
  if (pico_mode == 0.0) {
    // Shut down pump and fan
    PUMP.writeMicroseconds(900);
    pwm_set_gpio_level(FAN_PIN, 0);
    
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(150, 150, 0, 150));  // yellow
    pixels.show();
    
    while (pico_mode == 0.0) {   // Wait until move on
      if ((micros() - last_update_time) > update_dt) {
        update_temperature_and_mode();
        last_update_time = micros();
      }
    }
  }

  // Reset Pico
  if (pico_mode == 2.0) {
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(150, 0, 0, 150));  // green (grbw not rgbw!)
    pixels.show();
    sleep_ms(1000);
    
    watchdog_enable(1, 1);  // Enable watchdog
    while (true);           // Stall device to trigger watchdog and reboot
  }
}

// Device setup
void setup() {
  // Buffer to store raw reads
  uint8_t data[1];

  // Initialize LED
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(0, 0, 150, 150));  // Blue
  pixels.show();

  // Initialize Foot Contact pins
  gpio_init(LTOE_PIN);
  gpio_pull_up(LTOE_PIN);
  gpio_set_dir(LTOE_PIN, GPIO_IN);

  gpio_init(LHEEL_PIN);
  gpio_pull_up(LHEEL_PIN);
  gpio_set_dir(LHEEL_PIN, GPIO_IN);

  gpio_init(RTOE_PIN);
  gpio_pull_up(RTOE_PIN);
  gpio_set_dir(RTOE_PIN, GPIO_IN);

  gpio_init(RHEEL_PIN);
  gpio_pull_up(RHEEL_PIN);
  gpio_set_dir(RHEEL_PIN, GPIO_IN);

  // Initialize serial port
  Serial.begin(115200);
  Serial.setTimeout(1);

  //Initialize I2C port at 1000 kHz
  i2c_init(i2c, 1000 * 1000);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);

  // Read device ID to make sure that we can communicate with the ISM330DHCX
  bool IMU_OFFLINE = true;
  while (IMU_OFFLINE) {
    reg_read(i2c, ISM330DHCX_ADDR, REG_DEVID, data, 1);
    if (data[0] != ISM330DHCX_ID) {
      printf("ERROR: Could not communicate with ISM330DHCX\r\n");
      Serial.println(data[0]);
      
      // Turn off LED and wait for 0.5s
      pixels.clear();
      pixels.show();
      sleep_ms(500);
      
      // Turn on Blue and wait for 0.5s
      pixels.setPixelColor(0, pixels.Color(0, 0, 150, 150));
      pixels.show();
      sleep_ms(500);
    } else {
      // Found IMU
      IMU_OFFLINE = false;

      // LED quick blink 3 times and stay on
      for (int i = 0; i < 3; i++) {
        pixels.clear();
        pixels.show();
        sleep_ms(200);
        pixels.setPixelColor(0, pixels.Color(0, 0, 150, 150));
        pixels.show();
        sleep_ms(200);
      }
    }
  }

  // IMU setup
  setup_IMU();

  // IMU calibration
  if (calibration) {
    get_IMU_bias();  // Get IMU bias info in calibration mode
  } else {
    (*dt).floatingPoint          =  dt_default;
    (*a_bias_x).floatingPoint    =  0.0538750887;
    (*a_bias_y).floatingPoint    = -0.1528932005;
    (*a_bias_z).floatingPoint    =  0.0424703979;
    (*gyro_bias_x).floatingPoint =  0.0056394399;
    (*gyro_bias_y).floatingPoint = -0.0059941257;
    (*gyro_bias_z).floatingPoint = -0.0016747913;
  }

  // Start pump
  PUMP.attach(PUMP_PIN, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  PUMP.writeMicroseconds(900);        // send “stop” signal to pump to initialize.

  // Start fan
  static uint fan_freq = 25000;
  static uint fan_wrap = 101;
  gpio_set_function(FAN_PIN, GPIO_FUNC_PWM);
  static uint fan_slice_num = pwm_gpio_to_slice_num(FAN_PIN);
  pwm_config fan_config = pwm_get_default_config();
  float fan_divider = (float)clock_get_hz(clk_sys) / (fan_freq * fan_wrap);
  pwm_config_set_clkdiv(&fan_config, fan_divider);
  pwm_config_set_wrap(&fan_config, fan_wrap);
  pwm_init(fan_slice_num, &fan_config, true);
  pwm_set_gpio_level(FAN_PIN, 0);
  
  sleep_ms(4000);
}

// Main loop
void loop() {
  // read and send data to PC
  read_IMU();
  read_contact();
  dump();

  // get temperature info and pico mode from PC and send new signal to ESC
  if ((micros() - last_update_time) > update_dt) {
    update_temperature_and_mode();
    update_pump_and_fan_signal();
    last_update_time = micros();
  }

  // Pico mode operating
  pico_mode_setting();

  // check if overtime
  loop_time = micros() - t_final;
  if (loop_time > (*dt).floatingPoint) {
    error = error|0b00000001;
  } else {
    error = error&0b11111110;
  }

  // show error and contact
  LED_status();

  // loop time control
  while ((micros() - t_final) < (*dt).floatingPoint);  // Keep loop at dt
  t_final = micros();
}
