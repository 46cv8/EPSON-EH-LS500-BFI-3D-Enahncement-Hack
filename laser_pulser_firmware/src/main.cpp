/*
  ESP32 Firmware with NimBLE, EEPROM persistence, encrypted link enforcement,
  application-level authentication, and additional serial proxy functionality.
  ---------------------------------------------------------------------------
  This single‑file sketch now implements:
    • OTA updates via BLE (skeleton implementation for Arduino IDE updates)
    • A BLE GATT server exposing a set of global variables as characteristics (read‑write and read‑only)
    • Two operation modes: “3D Mode” (when a level change is detected on the 3D sync input within 10,000µs)
      and “Passthrough Mode” (otherwise).
    • Processing of several configurable GPIO signals:
         - GPIO_3D_SYNC_PIN: Monitors a 3.3V digital signal (3D sync signal)
         - GPIO_PWM_MCU_SIDE_INPUT_PIN: Monitors a PWM signal from the microcontroller (MCU) side
         - GPIO_PWM_LD_SIDE_OUTPUT_PIN: Outputs a gated/modified PWM signal to the LD (laser diode) side
         - GPIO_RXD_MCU_SIDE_INPUT_PIN: Monitors an RXD serial signal from the MCU side
         - GPIO_TXD_MCU_SIDE_OUTPUT_PIN: Outputs a modified TXD serial signal to the MCU side
         - GPIO_TXD_LD_SIDE_INPUT_PIN: Monitors a TXD serial signal from the LD (laser diode) side
         - GPIO_RXD_LD_SIDE_OUTPUT_PIN: Outputs a modified RXD serial signal to the LD (laser diode) side
    • Serial proxy functionality between the two UART ports:
         - Data received on the MCU side (from GPIO_RXD_MCU_SIDE_INPUT_PIN)
           is relayed (after a placeholder for modification) to the LD side (output on GPIO_RXD_LD_SIDE_OUTPUT_PIN).
         - Similarly, data received on the LD side (from GPIO_TXD_LD_SIDE_INPUT_PIN)
           is relayed (after a placeholder for modification) to the MCU side (output on GPIO_TXD_MCU_SIDE_OUTPUT_PIN).
    • Persistence of three updateable parameters (frame_delay, frame_duration,
      disable_pwm_during_active_interval) to EEPROM.
    • Enforcement that any connection uses an encrypted BLE link.
    • Application-level authentication using the secret "MYSECRET".
    • A "reset" endpoint that causes a hard restart of the ESP32.

  Trouble Shooting:
    * If after making a chance you get missing device characteristics try clearing your bluetooth cache.
    * bluetoothctl
    * devices
    * find "Device XX:XX:XX:XX:XX:XX ESP32_3D_PWM"
    * sudo find /var/lib/bluetooth/ -mindepth 1 -type f | grep cache
    * sudo rm -rf /var/lib/bluetooth/YY:YY:YY:YY:YY:YY/cache/XX:XX:XX:XX:XX:XX/
    * sudo systemctl restart bluetooth.service
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <EEPROM.h>
#include <Update.h>
#include <string.h>
#include "driver/uart.h" // For UART interrupt registration
#include "soc/uart_reg.h"

// =============================================================
// EEPROM configuration
#define EEPROM_SIZE 64

struct ConfigData
{
  char magic[9]; // "3D3D3D00" plus null terminator.
  uint8_t enable_pwm_mitm;
  unsigned long frame_delay;
  unsigned long frame_duration;
  uint8_t disable_pwm_during_active_interval;
  uint8_t enable_serial_mitm;
  uint16_t serial_mitm_brightness_override;
  uint8_t log_mcu_serial;
};

// =============================================================
// Configurable Pin Definitions
#define GPIO_3D_SYNC_PIN 36             // Input: 3D sync signal
#define GPIO_PWM_MCU_SIDE_INPUT_PIN 39  // Input: PWM Input signal
#define GPIO_RXD_MCU_SIDE_INPUT_PIN 34  // Input: RXD MCU Side Serial signal
#define GPIO_TXD_LD_SIDE_INPUT_PIN 35   // Input: TXD LD Side Serial signal
#define GPIO_PWM_LD_SIDE_OUTPUT_PIN 32  // Output: PWM Output signal
#define GPIO_RXD_LD_SIDE_OUTPUT_PIN 33  // Output: RXD LD Side Serial signal
#define GPIO_TXD_MCU_SIDE_OUTPUT_PIN 27 // Output: TXD MCU Side Serial signal (GPIO 25 didn't work for me for some reason was 60hz 2v-3.3v this may have been a defective esp32 https://forum.arduino.cc/t/disable-dac-pins-on-pin-25-and-26-on-esp32/883460/13 )

#define GPIO_DEBUG_1_PIN 14
#define GPIO_DEBUG_2_PIN 12
#define GPIO_DEBUG_3_PIN 13

// =============================================================
// Global Configuration Constants
#define THREE_D_MODE_TIMEOUT 20000UL // microseconds
#define SERIAL_QUEUE_MCU_TXD_INTERFACE 1
#define SERIAL_QUEUE_LD_RXD_INTERFACE 2
#define SERIAL_COMMAND_LD_RXD_FULL_PWM 1
#define SERIAL_COMMAND_LD_RXD_LOW_PWM 2

// =============================================================
// Global Variables
volatile uint8_t enable_pwm_mitm = 0;                       // 0: enable; 1: disable
volatile unsigned long frame_delay = 5000;                  // µs
volatile unsigned long frame_duration = 2000;               // µs
volatile uint8_t disable_pwm_during_active_interval = 0;    // 0: enable; 1: disable
volatile uint8_t enable_serial_mitm = 0;                    // 0: enable; 1: disable
volatile uint16_t serial_mitm_brightness_override = 0xABBF; // 0xA888 to 0xAFFF (?)
volatile uint8_t log_mcu_serial = 0;                        // 0: enable; 1: disable

volatile bool gpio_3d_mode_active = false;
volatile bool gpio_3d_signal_high = false;
volatile unsigned long gpio_3d_signal_last_timestamp = 0;

// Two interval buffers:
volatile unsigned long current_active_interval_start_time = 0;
volatile unsigned long current_active_interval_stop_time = 0;
volatile bool current_active_interval_set = false;
volatile unsigned long next_active_interval_start_time = 0;
volatile unsigned long next_active_interval_stop_time = 0;
volatile bool next_active_interval_set = false;
volatile bool in_active_interval = false;

// MITM State Variables
volatile bool serial_mitm_block_next_ld_txd = false;
volatile bool serial_mitm_replace_next_mcu_rxd = false;
volatile uint8_t serial_mitm_replace_next_mcu_rxd_value = false;

// Define the size of the serial log FIFO buffer.
#define SERIAL_LOG_BUFFER_SIZE 4096

// Delay 3D detection sa when the projector starts it outputs 30hz on the 3d sync line before things are initialized.
#define DELAY_3D_SYNC_DETECTION_ON_STARTUP_BY_MICROS 30000000

// Structure to hold each log entry.
struct SerialLogEntry
{
  unsigned long timestamp;
  uint8_t type; // 0: RXD, 1: TXD
  uint8_t data;
};
volatile SerialLogEntry serialLogBuffer[SERIAL_LOG_BUFFER_SIZE];
volatile int serialLogHead = 0;
volatile int serialLogCount = 0;

// =============================================================
// Ordered Serial Command Queue
#define SERIAL_QUEUE_MAX 16 // Maximum number of pending serial commands

// Structure for each serial queue entry.
struct SerialQueueEntry
{
  unsigned long send_after; // Time in micros after which the command can be sent
  uint8_t interface;        // 0: MCU TXD, 1: LD RXD
  int messageId;            // Identifier for the serial message to send
};

// Priority queue for outbound serial commands.
volatile SerialQueueEntry serialQueue[SERIAL_QUEUE_MAX];
volatile int serialQueueCount = 0;

// IRAM_ATTR helper function to add an entry to the serial queue in order.
IRAM_ATTR void addSerialQueueEntry(unsigned long send_after, uint8_t interface, int messageId)
{
  // Check if the queue is full
  if (serialQueueCount >= SERIAL_QUEUE_MAX)
  {
    // Queue full -- new entry is discarded.
    return;
  }
  // Find insertion index so that the entries remain ordered by send_after.
  int pos = 0;
  while (pos < serialQueueCount && serialQueue[pos].send_after <= send_after)
  {
    pos++;
  }
  // Shift entries right to make room for the new entry.
  for (int i = serialQueueCount; i > pos; i--)
  {
    memcpy((void *)&serialQueue[i], (const void *)&serialQueue[i - 1], sizeof(SerialQueueEntry));
  }
  // Insert the new queue entry.
  serialQueue[pos].send_after = send_after;
  serialQueue[pos].interface = interface;
  serialQueue[pos].messageId = messageId;
  serialQueueCount++;
}

// =============================================================
// IRAM_ATTR helper function to log a serial byte into the FIFO buffer.
IRAM_ATTR void logSerialByte(uint8_t type, uint8_t data)
{
  unsigned long now = micros();
  SerialLogEntry entry;
  entry.timestamp = now;
  entry.type = type;
  entry.data = data;
  // Use memcpy to avoid volatile assignment issues.
  memcpy((void *)&serialLogBuffer[serialLogHead], &entry, sizeof(entry));
  serialLogHead = (serialLogHead + 1) % SERIAL_LOG_BUFFER_SIZE;
  if (serialLogCount < SERIAL_LOG_BUFFER_SIZE)
  {
    serialLogCount++;
  }
  // If the buffer is full, the oldest entry is automatically discarded.
}

// =============================================================
// Authentication secret and flag
const char *SECRET = "MYSECRET";
bool isAuthenticated = false;

// =============================================================
// NimBLE UUIDs
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define OTA_CHAR_UUID "12345678-1234-1234-1234-1234567890ac"
#define COMBINED_UPDATE_UUID "12345678-1234-1234-1234-1234567890b1"
#define UPDATE_EEPROM_UUID "12345678-1234-1234-1234-1234567890b8"
#define COMBINED_READ_ONLY_UUID "12345678-1234-1234-1234-1234567890b7"
#define AUTH_CHAR_UUID "12345678-1234-1234-1234-1234567890b9"
#define RESET_UUID "12345678-1234-1234-1234-1234567890c0"
#define DUMP_MCU_SERIAL_LOG_UUID "12345678-1234-1234-1234-1234567890c1"

// =============================================================
// Define UART numbers corresponding to our HardwareSerial instances.
#define UART_MCU_NUM UART_NUM_1 // used by SerialMcu
#define UART_LD_NUM UART_NUM_2  // used by SerialLd

// =============================================================
// HardwareSerial instances for the serial proxy
// For MCU side: RX from GPIO_RXD_MCU_SIDE_INPUT_PIN, TX to GPIO_TXD_MCU_SIDE_OUTPUT_PIN
// For LD side:  RX from GPIO_TXD_LD_SIDE_INPUT_PIN, TX to GPIO_RXD_LD_SIDE_OUTPUT_PIN
HardwareSerial SerialMcu(UART_MCU_NUM);
HardwareSerial SerialLd(UART_LD_NUM);

// =============================================================
// Forward declarations
IRAM_ATTR void update_active_interval_status(unsigned long current_time);
IRAM_ATTR void update_intervals(unsigned long current_time);
IRAM_ATTR void check_3d_sync(unsigned long current_time);
IRAM_ATTR void check_pwm_input();

// =============================================================
// Forward declarations for the serial proxy processing function.
IRAM_ATTR void processMCUSerialProxy();
IRAM_ATTR void processLDSerialProxy();

// =============================================================
// NimBLE server and characteristics (global pointers)
NimBLEServer *pServer = nullptr;
NimBLEService *pService = nullptr;
NimBLECharacteristic *pOTACharacteristic = nullptr;
NimBLECharacteristic *pCombinedUpdateChar = nullptr;
NimBLECharacteristic *pUpdateEEPROMChar = nullptr;
NimBLECharacteristic *pReadOnlyCombinedChar = nullptr;
NimBLECharacteristic *pAuthChar = nullptr;
NimBLECharacteristic *pResetChar = nullptr;
NimBLECharacteristic *pDumpMcuSerialLogChar = nullptr; // Added for dump MCU serial log characteristic

// =============================================================
// Helper functions
String boolToStr(bool b)
{
  return (b ? "1" : "0");
}
String ulongToStr(unsigned long val)
{
  return String(val);
}

IRAM_ATTR void update_active_interval_status(unsigned long current_time)
{

  bool in_active_interval_new = false;
  bool gpio_3d_mode_active_new = gpio_3d_mode_active;

  // First, check if current interval is active and current time is within it.
  if (current_active_interval_set &&
      (current_time >= current_active_interval_start_time) &&
      (current_time <= current_active_interval_stop_time))
  {
    in_active_interval_new = true;
  }
  // Otherwise, if the next interval is set and current time falls within it,
  // promote next to current.
  else if (next_active_interval_set &&
           (current_time >= next_active_interval_start_time) &&
           (current_time <= next_active_interval_stop_time))
  {
    in_active_interval_new = true;
  }

  // If too much time has passed since the last 3D sync, disable 3D mode.
  if ((current_time - gpio_3d_signal_last_timestamp) > THREE_D_MODE_TIMEOUT)
  {
    gpio_3d_mode_active_new = false;
    digitalWrite(GPIO_DEBUG_1_PIN, false);
  }

  // Apply the gating (for simplicity, we mirror the PWM input when gated).
  if (gpio_3d_mode_active_new != gpio_3d_mode_active && !gpio_3d_mode_active_new)
  {
    digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, digitalRead(GPIO_PWM_MCU_SIDE_INPUT_PIN));

    if (enable_serial_mitm)
    {
      // Go low brightness as we are no longer PWM gating the signal
      // Consider using a ramp to decrease it
      SerialLd.write(0xA8);
      SerialLd.write(0x88);
      addSerialQueueEntry(current_time + 4750, SERIAL_QUEUE_LD_RXD_INTERFACE, SERIAL_COMMAND_LD_RXD_LOW_PWM);
    }
  }
  else if (in_active_interval_new != in_active_interval)
  {

    if (in_active_interval != disable_pwm_during_active_interval)
    {
      digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, digitalRead(GPIO_PWM_MCU_SIDE_INPUT_PIN));
    }
    else if (enable_pwm_mitm)
    {
      digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, LOW);
    }
    in_active_interval = in_active_interval_new;
  }
  if (gpio_3d_mode_active != gpio_3d_mode_active_new)
  {
    gpio_3d_mode_active = gpio_3d_mode_active_new;
  }
}

// =============================================================
// Interval update logic (called from the 3D sync ISR)
// This function maintains a buffer of two intervals.
IRAM_ATTR void update_intervals(unsigned long current_time)
{
  unsigned long new_start = current_time + frame_delay;
  unsigned long new_stop = new_start + frame_duration;

  if (!current_active_interval_set)
  {
    // No current interval: initialize current interval.
    current_active_interval_start_time = new_start;
    current_active_interval_stop_time = new_stop;
    current_active_interval_set = true;
    next_active_interval_set = false;
  }
  else
  {
    // If new sync occurs before current interval expires,
    // simply update the next interval.
    if (current_time < current_active_interval_stop_time)
    {
      next_active_interval_start_time = new_start;
      next_active_interval_stop_time = new_stop;
      next_active_interval_set = true;
    }
    else
    {
      // Current interval has expired.
      // If a next interval exists and is still valid (i.e. not expired),
      // promote it to current and then update next with the new interval.
      if (next_active_interval_set && (current_time < next_active_interval_stop_time))
      {
        current_active_interval_start_time = next_active_interval_start_time;
        current_active_interval_stop_time = next_active_interval_stop_time;
        // Then update next with the new interval.
        next_active_interval_start_time = new_start;
        next_active_interval_stop_time = new_stop;
        next_active_interval_set = true;
      }
      else
      {
        // Otherwise, just set current to the new interval.
        current_active_interval_start_time = new_start;
        current_active_interval_stop_time = new_stop;
        next_active_interval_set = false;
      }
    }
  }
}

IRAM_ATTR void check_3d_sync(unsigned long current_time)
{
  if (current_time > DELAY_3D_SYNC_DETECTION_ON_STARTUP_BY_MICROS)
  {
    bool gpio_3d_signal_high_new = digitalRead(GPIO_3D_SYNC_PIN);
    bool gpio_3d_mode_active_new = gpio_3d_mode_active;

    if (gpio_3d_signal_high_new != gpio_3d_signal_high)
    {
      update_intervals(current_time);
      gpio_3d_signal_high = gpio_3d_signal_high_new;
      gpio_3d_signal_last_timestamp = current_time;

      gpio_3d_mode_active_new = true;
      if (gpio_3d_mode_active_new != gpio_3d_mode_active)
      {
        gpio_3d_mode_active = gpio_3d_mode_active_new;
        if (enable_serial_mitm && gpio_3d_mode_active)
        {
          // Go max brightness as we are now PWM gating the signal
          // Consider using a ramp to increase it
          SerialLd.write((uint8_t)(0xAF & (serial_mitm_brightness_override >> 8)));
          SerialLd.write((uint8_t)(0xFF & serial_mitm_brightness_override));
          addSerialQueueEntry(current_time + 4750, SERIAL_QUEUE_LD_RXD_INTERFACE, SERIAL_COMMAND_LD_RXD_FULL_PWM);
        }
        digitalWrite(GPIO_DEBUG_1_PIN, true);
      }
    }
  }
}

IRAM_ATTR void check_pwm_input()
{
  if (in_active_interval != disable_pwm_during_active_interval || !gpio_3d_mode_active || !enable_pwm_mitm)
  {
    digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, digitalRead(GPIO_PWM_MCU_SIDE_INPUT_PIN));
  }
  else
  {
    digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, LOW);
  }
}

// =============================================================
// MCU Serial Proxy Processing Function
// This function reads data from the MCU serial port and forwards it appropriately.
// It is called both from the main loop and from the UART interrupt handlers.
IRAM_ATTR void processMCUSerialProxy()
{
  // Process data from MCU side -> LD side
  while (SerialMcu.available())
  {
    int incomingByte = SerialMcu.read();
    bool blockProxy = false;
    if (log_mcu_serial)
    {
      logSerialByte(0, incomingByte); // Log RXD data.
    }
    if (enable_serial_mitm && gpio_3d_mode_active)
    {
      if ((incomingByte & 0xA8) == 0xA8)
      {
        SerialLd.write((uint8_t)(0xAF & (serial_mitm_brightness_override >> 8)));
        SerialMcu.write(incomingByte);
        serial_mitm_replace_next_mcu_rxd = true;
        serial_mitm_replace_next_mcu_rxd_value = (uint8_t)(0xFF & serial_mitm_brightness_override);
        blockProxy = true;
      }
      else if ((incomingByte & 0xC8) == 0xC8)
      {
        SerialLd.write(0xCF);
        SerialMcu.write(incomingByte);
        serial_mitm_replace_next_mcu_rxd = true;
        serial_mitm_replace_next_mcu_rxd_value = 0xFF;
        blockProxy = true;
      }
      else if (serial_mitm_replace_next_mcu_rxd)
      {
        SerialLd.write(serial_mitm_replace_next_mcu_rxd_value);
        SerialMcu.write(incomingByte);
        serial_mitm_replace_next_mcu_rxd = false;
        blockProxy = true;
      }
    }
    if (!blockProxy)
    {
      SerialLd.write(incomingByte);
    }
  }
}

// =============================================================
// LD Serial Proxy Processing Function
// This function reads data from the LD serial port and forwards it appropriately.
// It is called both from the main loop and from the UART interrupt handlers.
IRAM_ATTR void processLDSerialProxy()
{
  // Process data from LD side -> MCU side
  while (SerialLd.available())
  {
    int incomingByte = SerialLd.read();
    bool blockProxy = false;
    if (log_mcu_serial)
    {
      logSerialByte(1, incomingByte); // Log TXD data.
    }
    if (enable_serial_mitm && gpio_3d_mode_active)
    {
      if ((incomingByte & 0xA8) == 0xA8)
      {
        serial_mitm_block_next_ld_txd = true;
        blockProxy = true;
      }
      else if ((incomingByte & 0xC8) == 0xC8)
      {
        serial_mitm_block_next_ld_txd = true;
        blockProxy = true;
      }
      else if (serial_mitm_block_next_ld_txd)
      {
        serial_mitm_block_next_ld_txd = false;
        blockProxy = true;
      }
    }
    if (!blockProxy)
    {
      SerialMcu.write(incomingByte);
    }
  }
}

// =============================================================
// BLE-related classes (server callbacks, characteristic callbacks, etc.)
class MyServerCallbacks : public NimBLEServerCallbacks
{
public:
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override
  {
    Serial.println("Client connected.");
  }
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override
  {
    Serial.println("Client disconnected.");
    isAuthenticated = false;
    NimBLEDevice::startAdvertising();
  }
};

class AuthCallback : public NimBLECharacteristicCallbacks
{
public:
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    std::string value = pCharacteristic->getValue();
    if (value == SECRET)
    {
      isAuthenticated = true;
      Serial.println("Authentication successful.");
    }
    else
    {
      isAuthenticated = false;
      Serial.println("Authentication failed.");
    }
  }
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    const char *resp = isAuthenticated ? "Authenticated" : "Not authenticated";
    pCharacteristic->setValue((uint8_t *)resp, strlen(resp));
  }
};

// -----------------------------------------------------------------
// The following class has been renamed from AuthReadCallback to CombinedReadCallback
// and its onRead logic now includes the BLE update string logic that was previously
// handled by updateBLECharacteristics().
class CombinedReadCallback : public NimBLECharacteristicCallbacks
{
public:
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    char buf[128];
    int len;
    if (!isAuthenticated)
    {
      const char *resp = "Not authenticated";
      len = strlen(resp);
      memcpy(buf, resp, len);
    }
    else
    {
      len = snprintf(buf, sizeof(buf),
                     "{\"g3\":\"%s\","
                     "\"gs\":\"%s\","
                     "\"gt\":\"%s\","
                     "\"cs\":\"%s\","
                     "\"ce\":\"%s\"}",
                     boolToStr(gpio_3d_mode_active).c_str(),
                     boolToStr(gpio_3d_signal_high).c_str(),
                     ulongToStr(gpio_3d_signal_last_timestamp).c_str(),
                     ulongToStr(current_active_interval_start_time).c_str(),
                     ulongToStr(current_active_interval_stop_time).c_str());
    }
    pCharacteristic->setValue((uint8_t *)buf, len);
  }
};

class CombinedUpdateCallback : public NimBLECharacteristicCallbacks
{
public:
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    if (!isAuthenticated)
    {
      const char *resp = "Not authenticated";
      pCharacteristic->setValue((uint8_t *)resp, strlen(resp));
      return;
    }
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "{\"pwm\":%u,\"fd\":%lu,\"fr\":%lu,\"di\":%u,\"es\":%u,\"sb\":\"%04X\",\"ls\":%u}",
                       enable_pwm_mitm, frame_delay, frame_duration, disable_pwm_during_active_interval,
                       enable_serial_mitm, serial_mitm_brightness_override, log_mcu_serial);
    pCharacteristic->setValue((uint8_t *)buf, len);
  }
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    if (!isAuthenticated)
    {
      Serial.println("Write rejected: not authenticated");
      return;
    }
    std::string value = pCharacteristic->getValue();
    unsigned long new_frame_delay, new_frame_duration;
    int new_disable_pwm, new_enable_serial, new_log_mcu, new_enable_pwm_mitm;
    unsigned int new_serial_mitm_brightness_override_int;
    int matched = sscanf(value.c_str(),
                         "{\"pwm\":%d,\"fd\":%lu,\"fr\":%lu,\"di\":%d,\"es\":%d,\"sb\":\"%4x\",\"ls\":%d}",
                         &new_enable_pwm_mitm, &new_frame_delay, &new_frame_duration, &new_disable_pwm,
                         &new_enable_serial, &new_serial_mitm_brightness_override_int, &new_log_mcu);
    if (matched == 7)
    {
      // Validate the brightness override range (A888 to AFFF)
      if (new_serial_mitm_brightness_override_int < 0xA888 || new_serial_mitm_brightness_override_int > 0xAFFF)
      {
        Serial.println("Brightness override value out of range. Must be between A888 and AFFF.");
        return;
      }
      frame_delay = new_frame_delay;
      frame_duration = new_frame_duration;
      disable_pwm_during_active_interval = (uint8_t)new_disable_pwm;
      enable_serial_mitm = (new_enable_serial != 0);
      log_mcu_serial = (new_log_mcu != 0);
      enable_pwm_mitm = (new_enable_pwm_mitm != 0);
      serial_mitm_brightness_override = (uint16_t)new_serial_mitm_brightness_override_int;
      Serial.println("Combined parameters updated:");
      Serial.printf("  enable_pwm_mitm: %u\n", enable_pwm_mitm);
      Serial.printf("  frame_delay: %lu\n", frame_delay);
      Serial.printf("  frame_duration: %lu\n", frame_duration);
      Serial.printf("  disable_pwm_during_active_interval: %u\n", disable_pwm_during_active_interval);
      Serial.printf("  enable_serial_mitm: %u\n", enable_serial_mitm);
      Serial.printf("  serial_mitm_brightness_override: 0x%04X\n", serial_mitm_brightness_override);
      Serial.printf("  log_mcu_serial: %u\n", log_mcu_serial);
    }
    else
    {
      Serial.println("Failed to parse combined parameters.");
    }
  }
};

class ResetCallback : public NimBLECharacteristicCallbacks
{
public:
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    if (!isAuthenticated)
    {
      Serial.println("Reset command rejected: not authenticated");
      return;
    }
    Serial.println("Reset command received. Restarting ESP32...");
    ESP.restart();
  }
};

// Updated DumpMcuSerialLogCallback
// data have been built using only complete RXD/TXD groups. (A group is only “complete”
// if a new log entry for that type was seen that is at least 1000 µs later, so that the
// previous group is not “midway”.) In that case, the callback will update the log FIFO’s
// serialLogCount to “consume” (remove) only the entries that were sent to the client.
class DumpMcuSerialLogCallback : public NimBLECharacteristicCallbacks
{
public:
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    if (!isAuthenticated)
    {
      const char *resp = "Not authenticated";
      pCharacteristic->setValue((uint8_t *)resp, strlen(resp));
      return;
    }

    // We work with the circular log buffer in FIFO order.
    // Compute the tail index.
    int tail = (serialLogHead - serialLogCount + SERIAL_LOG_BUFFER_SIZE) % SERIAL_LOG_BUFFER_SIZE;

    // Define our temporary structure for a "group" (a set of log entries for one type).
    struct SerialLogGroup
    {
      unsigned long groupTime; // timestamp of first entry in the group
      uint8_t type;            // 0 = RXD, 1 = TXD
      String groupData;        // concatenated hex values
      int endRelativeIdx;      // the relative FIFO index (0-based) of the last log in this group
    };

    // We will collect groups only for those log entries that are complete.
    // (A group is complete only if a new entry for that type started a new group.)
    const int maxGroups = 50;
    SerialLogGroup groups[maxGroups];
    int groupCount = 0;

    // For each type we keep a temporary group that is “in progress”.
    bool rxGroupActive = false, txGroupActive = false;
    unsigned long rxGroupTime = 0, txGroupTime = 0;
    String rxGroupData = "";
    String txGroupData = "";
    int rxGroupEnd = -1, txGroupEnd = -1;

    // Process the FIFO entries in chronological order (from tail).
    // i will be the relative index (0 ... serialLogCount-1)
    int i;
    for (i = 0; i < serialLogCount && groupCount < maxGroups; i++)
    {
      int idx = (tail + i) % SERIAL_LOG_BUFFER_SIZE;
      SerialLogEntry entry;
      // Copy the entry from volatile memory
      memcpy(&entry, (void *)&serialLogBuffer[idx], sizeof(entry));

      if (entry.type == 0)
      { // RXD entry
        if (!rxGroupActive)
        {
          // Start a new RX group.
          rxGroupActive = true;
          rxGroupTime = entry.timestamp;
          rxGroupData = "0x" + String(entry.data, HEX);
          rxGroupEnd = i;
        }
        else
        {
          // Already in an RX group. Check the time gap.
          if ((entry.timestamp - rxGroupTime) < 1000)
          {
            // Same group: append this entry.
            rxGroupData += " 0x" + String(entry.data, HEX);
            rxGroupEnd = i;
          }
          else
          {
            // The gap indicates that the RX group is complete.
            if (groupCount < maxGroups)
            {
              groups[groupCount].groupTime = rxGroupTime;
              groups[groupCount].type = 0;
              groups[groupCount].groupData = rxGroupData;
              groups[groupCount].endRelativeIdx = rxGroupEnd;
              groupCount++;
            }
            // Start a new RX group with the current entry.
            rxGroupTime = entry.timestamp;
            rxGroupData = "0x" + String(entry.data, HEX);
            rxGroupEnd = i;
          }
        }
      }
      else
      { // TXD entry (entry.type == 1)
        if (!txGroupActive)
        {
          txGroupActive = true;
          txGroupTime = entry.timestamp;
          txGroupData = "0x" + String(entry.data, HEX);
          txGroupEnd = i;
        }
        else
        {
          if ((entry.timestamp - txGroupTime) < 1000)
          {
            txGroupData += " 0x" + String(entry.data, HEX);
            txGroupEnd = i;
          }
          else
          {
            if (groupCount < maxGroups)
            {
              groups[groupCount].groupTime = txGroupTime;
              groups[groupCount].type = 1;
              groups[groupCount].groupData = txGroupData;
              groups[groupCount].endRelativeIdx = txGroupEnd;
              groupCount++;
            }
            txGroupTime = entry.timestamp;
            txGroupData = "0x" + String(entry.data, HEX);
            txGroupEnd = i;
          }
        }
      }
    }
    // Do NOT flush the "active" (incomplete) groups:
    // (i.e. if either rxGroupActive or txGroupActive is still true, we ignore that group)

    // At this point our array 'groups' holds all complete groups from the FIFO.
    // Sort these groups by their groupTime so that RXD and TXD lines are interleaved in order.
    for (int a = 0; a < groupCount - 1; a++)
    {
      for (int b = a + 1; b < groupCount; b++)
      {
        if (groups[b].groupTime < groups[a].groupTime)
        {
          SerialLogGroup temp = groups[a];
          groups[a] = groups[b];
          groups[b] = temp;
        }
      }
    }

    // Build CSV lines from the sorted groups until we accumulate at least 256 bytes.
    String csv = "";
    int flushBoundary = -1; // relative index (from tail) of the last log entry to flush.
    for (int a = 0; a < groupCount; a++)
    {
      String line = String(groups[a].groupTime) + ",";
      line += (groups[a].type == 0 ? "RXD" : "TXD");
      line += ",";
      line += groups[a].groupData;
      line += "\n";

      // If adding this group would put us over the target length, include it and then stop.
      if (csv.length() + line.length() >= 128)
      {
        csv += line;
        flushBoundary = groups[a].endRelativeIdx; // note: endRelativeIdx is relative to tail
        break;
      }
      else
      {
        csv += line;
        flushBoundary = groups[a].endRelativeIdx; // update flush boundary as we add each complete group
      }
    }

    // Remove from the log buffer only the entries that were included.
    // flushBoundary is a relative index (0-based) counted from tail, so
    // we have consumed flushBoundary+1 entries.
    int entriesToConsume = flushBoundary + 1;
    noInterrupts();
    if (entriesToConsume <= serialLogCount)
    {
      serialLogCount -= entriesToConsume;
    }
    else
    {
      serialLogCount = 0;
    }
    interrupts();

    // Finally send the CSV string to the client.
    pCharacteristic->setValue((uint8_t *)csv.c_str(), csv.length());
  }
};

class UpdateEEPROMCallback : public NimBLECharacteristicCallbacks
{
public:
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    if (!isAuthenticated)
    {
      Serial.println("EEPROM update rejected: not authenticated");
      return;
    }
    Serial.println("Updating EEPROM with current configuration...");
    ConfigData cfg;
    strcpy(cfg.magic, "3D3D3D00");
    cfg.enable_pwm_mitm = enable_pwm_mitm;
    cfg.frame_delay = frame_delay;
    cfg.frame_duration = frame_duration;
    cfg.disable_pwm_during_active_interval = disable_pwm_during_active_interval;
    cfg.enable_serial_mitm = enable_serial_mitm;
    cfg.serial_mitm_brightness_override = serial_mitm_brightness_override;
    cfg.log_mcu_serial = log_mcu_serial;
    EEPROM.put(0, cfg);
    EEPROM.commit();
    Serial.println("EEPROM updated.");
  }
};

class OTAUpdateCallback : public NimBLECharacteristicCallbacks
{
public:
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override
  {
    std::string data = pCharacteristic->getValue();
    if (data == "OTA_START")
    {
      Serial.println("OTA update started.");
      if (Update.begin(UPDATE_SIZE_UNKNOWN))
      {
        Serial.println("Update.begin() succeeded.");
      }
      else
      {
        Serial.println("Update.begin() failed.");
      }
      return;
    }
    if (data == "OTA_END")
    {
      Serial.println("OTA update ending...");
      if (Update.end(true))
      {
        Serial.println("OTA update completed successfully. Restarting...");
        ESP.restart();
      }
      else
      {
        Serial.printf("OTA update failed. Error: %d\n", Update.getError());
      }
      return;
    }
    size_t len = data.length();
    if (len > 0)
    {
      size_t written = Update.write((uint8_t *)data.data(), len);
      if (written != len)
      {
        Serial.println("OTA update write error.");
      }
      else
      {
        Serial.printf("OTA update wrote %d bytes.\n", written);
      }
    }
  }
};

// =============================================================
// Setup function.
void setup()
{
  // Initialize global variables
  enable_pwm_mitm = 0;                      // 0: enable; 1: disable
  frame_delay = 4000;                       // µs
  frame_duration = 4000;                    // µs
  disable_pwm_during_active_interval = 0;   // 0: enable; 1: disable
  enable_serial_mitm = 0;                   // 0: enable; 1: disable
  serial_mitm_brightness_override = 0xABBF; // 0xA888 to 0xAFFF (?)
  log_mcu_serial = 0;                       // 0: enable; 1: disable
  gpio_3d_mode_active = false;
  gpio_3d_signal_high = false;
  gpio_3d_signal_last_timestamp = 0;
  current_active_interval_start_time = 0;
  current_active_interval_stop_time = 0;
  current_active_interval_set = false;
  next_active_interval_start_time = 0;
  next_active_interval_stop_time = 0;
  next_active_interval_set = false;
  in_active_interval = false;
  serial_mitm_block_next_ld_txd = false;
  serial_mitm_replace_next_mcu_rxd = false;
  serial_mitm_replace_next_mcu_rxd_value = 0;
  serialLogHead = 0;
  serialLogCount = 0;
  serialQueueCount = 0;

  // Configure GPIO pins.
  pinMode(GPIO_3D_SYNC_PIN, INPUT);
  pinMode(GPIO_PWM_MCU_SIDE_INPUT_PIN, INPUT);
  pinMode(GPIO_RXD_MCU_SIDE_INPUT_PIN, INPUT);
  pinMode(GPIO_TXD_MCU_SIDE_OUTPUT_PIN, OUTPUT);
  pinMode(GPIO_PWM_LD_SIDE_OUTPUT_PIN, OUTPUT);
  digitalWrite(GPIO_PWM_LD_SIDE_OUTPUT_PIN, HIGH);
  pinMode(GPIO_RXD_LD_SIDE_OUTPUT_PIN, OUTPUT);
  pinMode(GPIO_TXD_LD_SIDE_INPUT_PIN, INPUT);

  pinMode(GPIO_DEBUG_1_PIN, OUTPUT);
  digitalWrite(GPIO_DEBUG_1_PIN, LOW);
  pinMode(GPIO_DEBUG_2_PIN, OUTPUT);
  digitalWrite(GPIO_DEBUG_2_PIN, LOW);
  pinMode(GPIO_DEBUG_3_PIN, OUTPUT);
  digitalWrite(GPIO_DEBUG_3_PIN, LOW);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting ESP32 firmware...");
  Serial.print("CPU Freq Mhz ");
  Serial.println(ESP.getCpuFreqMHz());

  EEPROM.begin(EEPROM_SIZE);
  ConfigData cfg;
  EEPROM.get(0, cfg);
  if (strcmp(cfg.magic, "3D3D3D00") == 0)
  {
    enable_pwm_mitm = cfg.enable_pwm_mitm;
    frame_delay = cfg.frame_delay;
    frame_duration = cfg.frame_duration;
    disable_pwm_during_active_interval = cfg.disable_pwm_during_active_interval;
    enable_serial_mitm = cfg.enable_serial_mitm;
    serial_mitm_brightness_override = cfg.serial_mitm_brightness_override;
    log_mcu_serial = cfg.log_mcu_serial;
  }
  else
  {
    Serial.println("No valid EEPROM configuration found, using defaults.");
  }
  Serial.println("Loaded configuration:");
  Serial.printf("  enable_pwm_mitm: %lu\n", enable_pwm_mitm);
  Serial.printf("  frame_delay: %lu\n", frame_delay);
  Serial.printf("  frame_duration: %lu\n", frame_duration);
  Serial.printf("  disable_pwm_during_active_interval: %u\n", disable_pwm_during_active_interval);
  Serial.printf("  enable_serial_mitm: %u\n", enable_serial_mitm);
  Serial.printf("  serial_mitm_brightness_override: 0x%04X\n", serial_mitm_brightness_override);
  Serial.printf("  log_mcu_serial: %u\n", log_mcu_serial);

  // Initialize additional serial ports for the proxy.
  // MCU Side: RX from GPIO_RXD_MCU_SIDE_INPUT_PIN, TX to GPIO_TXD_MCU_SIDE_OUTPUT_PIN
  // SerialMcu.setRxTimeout(10);
  // SerialMcu.setRxBufferSize(1);
  SerialMcu.begin(38400, SERIAL_8E1, GPIO_RXD_MCU_SIDE_INPUT_PIN, GPIO_TXD_MCU_SIDE_OUTPUT_PIN);
  // SerialMcu.begin(38400, SERIAL_8E1, GPIO_RXD_MCU_SIDE_INPUT_PIN, GPIO_TXD_MCU_SIDE_OUTPUT_PIN, false, 20000UL, (uint8_t)1U);
  //  LD Side: RX from GPIO_TXD_LD_SIDE_INPUT_PIN, TX to GPIO_RXD_LD_SIDE_OUTPUT_PIN
  // SerialLd.setRxTimeout(10);
  // SerialLd.setRxBufferSize(1);
  SerialLd.begin(38400, SERIAL_8E1, GPIO_TXD_LD_SIDE_INPUT_PIN, GPIO_RXD_LD_SIDE_OUTPUT_PIN);
  // SerialLd.begin(38400, SERIAL_8E1, GPIO_TXD_LD_SIDE_INPUT_PIN, GPIO_RXD_LD_SIDE_OUTPUT_PIN, false, 20000UL, (uint8_t)1U);

  // Config UART to have minimum latency
  // The above is insufficient as HardwareSerial.begin(rxfifo_full_thrhd) doesn't actually map to rxfifo_full_thresh
  // https://esp32.com/viewtopic.php?t=16380
  // https://esp32.com/viewtopic.php?t=3751
  //*
  uart_intr_config_t uart_intr;
  uart_intr.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_RXFIFO_TOUT_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M | UART_BRK_DET_INT_ENA_M | UART_PARITY_ERR_INT_ENA_M;
  uart_intr.rxfifo_full_thresh = 1;        // UART_FULL_THRESH_DEFAULT,  //120 default!! aghh! need receive 120 chars before we see them
  uart_intr.rx_timeout_thresh = 10;        // UART_TOUT_THRESH_DEFAULT,  //10 works well for my short messages I need send/receive
  uart_intr.txfifo_empty_intr_thresh = 10; // UART_EMPTY_THRESH_DEFAULT
  uart_intr_config(UART_MCU_NUM, &uart_intr);
  uart_intr_config(UART_LD_NUM, &uart_intr);
  //*/

  NimBLEDevice::init("ESP32_3D_PWM");
  NimBLEDevice::setSecurityAuth(true, false, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);

  pOTACharacteristic = pService->createCharacteristic(OTA_CHAR_UUID, NIMBLE_PROPERTY::WRITE);
  pOTACharacteristic->setCallbacks(new OTAUpdateCallback());

  pCombinedUpdateChar = pService->createCharacteristic(COMBINED_UPDATE_UUID,
                                                       NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
  pCombinedUpdateChar->setCallbacks(new CombinedUpdateCallback());

  pUpdateEEPROMChar = pService->createCharacteristic(UPDATE_EEPROM_UUID, NIMBLE_PROPERTY::WRITE);
  pUpdateEEPROMChar->setCallbacks(new UpdateEEPROMCallback());

  pReadOnlyCombinedChar = pService->createCharacteristic(COMBINED_READ_ONLY_UUID, NIMBLE_PROPERTY::READ);
  pReadOnlyCombinedChar->setCallbacks(new CombinedReadCallback());

  pAuthChar = pService->createCharacteristic(AUTH_CHAR_UUID,
                                             NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
  pAuthChar->setCallbacks(new AuthCallback());
  const char *authInit = "Not authenticated";
  pAuthChar->setValue((uint8_t *)authInit, strlen(authInit));

  pResetChar = pService->createCharacteristic(RESET_UUID, NIMBLE_PROPERTY::WRITE);
  pResetChar->setCallbacks(new ResetCallback());

  pDumpMcuSerialLogChar = pService->createCharacteristic(DUMP_MCU_SERIAL_LOG_UUID, NIMBLE_PROPERTY::READ);
  pDumpMcuSerialLogChar->setCallbacks(new DumpMcuSerialLogCallback());

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  NimBLEAdvertisementData scanResponse;
  scanResponse.setName("ESP32_3D_PWM");
  pAdvertising->setScanResponseData(scanResponse);

  NimBLEDevice::startAdvertising();
  Serial.println("NimBLE service started, advertising now...");
}

// =============================================================
// Main loop.
void loop()
{
  // Check the 3D sync signal.
  unsigned long current_time = micros();
  check_3d_sync(current_time);
  check_pwm_input();
  update_active_interval_status(current_time);

  // Check for serial data.
  processMCUSerialProxy();
  processLDSerialProxy();

  // Process Entries on Serial Queue
  while (serialQueueCount > 0 && serialQueue[0].send_after <= current_time)
  {
    // Retrieve the next scheduled serial command.
    SerialQueueEntry entry;
    memcpy(&entry, (const void *)&serialQueue[0], sizeof(SerialQueueEntry));
    // Remove the processed entry from the queue by shifting remaining elements left.
    for (int i = 0; i < serialQueueCount - 1; i++)
    {
      memcpy((void *)&serialQueue[i], (const void *)&serialQueue[i + 1], sizeof(SerialQueueEntry));
    }
    serialQueueCount--;

    // Dispatch the serial command based on the selected interface.
    if (entry.interface == SERIAL_QUEUE_MCU_TXD_INTERFACE)
    {
      switch (entry.messageId)
      {
      case 1:
        // SerialMcu.write(entry.messageId);
        break;
      default:
        break;
      }
    }
    else if (entry.interface == SERIAL_QUEUE_LD_RXD_INTERFACE)
    {
      switch (entry.messageId)
      {
      case SERIAL_COMMAND_LD_RXD_FULL_PWM:
        SerialLd.write(0xCF);
        SerialLd.write(0xFF);
        break;
      case SERIAL_COMMAND_LD_RXD_LOW_PWM:
        SerialLd.write(0xCB);
        SerialLd.write(0xE7);
        break;
      default:
        break;
      }
    }
  }
}
