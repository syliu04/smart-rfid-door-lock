/**
 * @file SmartRFIDDoorLock.ino
 * @brief Smart RFID Door Lock system built on the ESP32 using FreeRTOS.
 *
 * Access is granted or denied depending on the identity of the RFID card and 
 * the current time read from a DS3231 RTC. An IR remote changes between three
 * modes: Office Hours, Locked, and Free. Decisions for access are indicated
 * using green or red LEDs, a buzzer, and an I2C LCD display.
 * Tasks are divided between both ESP32 cores and execute parallelly.
 *
 * @section version_info Version Information
 * Version: 2.0
 *
 * @section authors Authors
 * - Aditi Manjunath (aditim4)
 * - ShengYao Liu (sliu1229)
 *
 * @section dates Dates
 * - Original Date: 03/04/2026
 * - Updated Date: 03/11/2026
 *
 * @section changes Changes
 * Hardware components (RFID, IR receiver, RTC) are working
 * on the board. LCD task added displaying output.
 */

/* ==================== INCLUDES ==================== */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Wire.h>
#include <RTClib.h>
#include <IRremote.hpp>
#include <LiquidCrystal_I2C.h>
#include "esp_task_wdt.h"

/* ==================== MACROS ==================== */

// ── RFID (SPI) ──
#define RFID_SS 10 ///< RC522 RFID module SS pin(Slave Select)
#define RFID_RST 9 ///< RC522 RFID module Reset pin

// ── bus pins: SPI (defined especially for ESP32-S3) ──
#define SPI_SCK 12 ///< clock pin SPI
#define SPI_MISO 13 ///< MISO pin SPI
#define SPI_MOSI 11 ///< MOSI pin SPI

// ── I2C bus pins (RTC and LCD)──
#define SDA_PIN 21 ///< I2C SDA pin
#define SCL_PIN 47 ///< I2C SCL pin

// ── Output peripherals ──
#define ACCESS_LED 6 ///< Green LED: access granted when light turns on
#define RED_LED 7 ///< Red LED: access denied when lights turn on
#define BUZZER 8 ///< Piezo buzzer: beeps if grant or deny

// ── IR receiver ──
#define IR_PIN 4 ///< GPIO that's connected to IR receiver data output

/**
 * @brief IR remote button command codes.
 *
 * The hex values are the NEC command byte that will be
 * decoded by the IRremote library.
 */
#define IR_BTN_OFFICE 0x45 ///< Changes to MODE_OFFICE
#define IR_BTN_LOCKED 0x46 ///< Changes to MODE_LOCKED
#define IR_BTN_FREE 0x47 ///< Changes to MODE_FREE

// ── Office-hours window ──
const int START_HOUR = 9; ///< Access allowed from 9 AM
const int END_HOUR = 17; ///< Access allowed up to 5 PM (non-inclusive)

// ── FreeRTOS task periods ──
const TickType_t RFID_PERIOD = pdMS_TO_TICKS(20); ///< Checks for RFID cards 50 times per second
const TickType_t RTC_PERIOD = pdMS_TO_TICKS(1000); ///< 1 Hz RTC rate of update
const TickType_t UI_PERIOD = pdMS_TO_TICKS(20); ///< 50 Hz task period for UI
const TickType_t IR_PERIOD = pdMS_TO_TICKS(15.6); ///< Checks the IR receiver about 64 times per second

// ── RFID cooldown ──
const TickType_t RFID_COOLDOWN_TICKS = pdMS_TO_TICKS(2000);  ///< Cooldown after a scan

/* ==================== TYPES ==================== */

/**
 * @brief Selectable operating modes using IR remote.
 */
enum Mode
{
  MODE_OFFICE, ///< Access is granted only to authorized UID during office hours
  MODE_LOCKED, ///< Access is denied irrespective of UID or time
  MODE_FREE ///< Access is granted to the card at any time
};

/**
 * @brief Event types placed on the event queue.
 */
enum EventType
{
  EVT_RFID ///< RFID card was scanned
};

/**
 * @brief RFIDTask messages ControlTask using eventQueue.
 */
struct EventMsg
{
  EventType type; ///< EVT_RFID for now, can be used for other events as well
  bool uidOk; ///< true if the scanned UID matches the allowed UID
};

/**
 * @brief Actions that ControlTask resolves to and sends downstream.
 */
enum UiAction
{
  UI_GRANT, ///< Access was granted
  UI_DENY ///< Access was denied
};

/**
 * @brief Message sent from ControlTask to UITask and LCDTask.
 */
struct UiMsg
{
  UiAction action; ///< Whether to show a grant or deny response
};

/* ==================== GLOBAL VARIABLES / CONSTANTS ==================== */

MFRC522 rfid(RFID_SS, RFID_RST); ///< Instance of RC522 RFID reader
RTC_DS3231 rtc; ///< Instance of DS3231 real-time clock
LiquidCrystal_I2C lcd(0x27, 16, 2); ///< Instance of 16x2 I2C LCD

QueueHandle_t eventQueue = NULL; ///< RFIDTask to ControlTask queue
QueueHandle_t uiQueue = NULL; ///< ControlTask to UITask queue
QueueHandle_t lcdQueue = NULL; ///< ControlTask to LCDTask queue

SemaphoreHandle_t timeMutex = NULL; ///< protects nowHour / nowMinute
SemaphoreHandle_t modeMutex = NULL; ///< protects currentMode

volatile Mode currentMode = MODE_OFFICE; ///< Active operating mode
volatile int  nowHour = 0; ///< Present hour read from RTC
volatile int  nowMinute = 0; ///< Present minute read from RTC

/**
 * @brief Ready flag of RFID shared between RFIDTask and LCDTask.
 *
 * RFIDTask assigns this to false after a scan and changes it to true after cooldown.
 * LCDTask uses this to decide the reverting of row 1 from GRANT/DENY back to
 * RFID ready.
 */
volatile bool rfidReady = true;

/* ==================== FUNCTION PROTOTYPES ==================== */

bool uidMatch(byte *uid, byte size);
bool officeHours(int h);
const char* modeName(Mode m);
void grantAccess();
void denyAccess();

void RFIDTask(void *p);
void IRTask(void *p);
void RTCTask(void *p);
void ControlTask(void *p);
void UITask(void *p);
void LCDTask(void *p);

/* ==================== HELPER FUNCTIONS ==================== */

byte allowedUID[4] = {0xDE, 0x7D, 0xE1, 0x00};  ///< UID of the RFID card that is authorized

/**
 * @brief Compares a scanned RFID UID against the authorized UID.
 *
 * This function compares the first 4 bytes of a scanned UID with the
 * allowedUID array which is hardcoded.
 *
 * @param uid Pointer to the byte array of the scanned card UID.
 * @param size Number of bytes in the scanned UID.
 * @return true if the first 4 UID bytes equal the authorized UID.
 * @return false if not.
 */
bool uidMatch(byte *uid, byte size) {
  if (size < 4) return false;
  for (int i = 0; i < 4; i++) {
    if (uid[i] != allowedUID[i]) return false;
  }
  return true;
}

/**
 * @brief Decides if the given hour is in the office hours.
 *
 * The office-hours window = [START_HOUR, END_HOUR).
 *
 * @param h Present hour in the 24-hour format.
 * @return true if the hour is in between office hours.
 * @return false if not.
 */
bool officeHours(int h) {
  return (h >= START_HOUR && h < END_HOUR);
}

/**
 * @brief Changes a Mode enum value into a string that's human readable.
 *
 * Used for a Serial debugging output and LCD display text.
 *
 * @param m The Mode value to convert.
 * @return Constant C-string matching the mode.
 */
const char* modeName(Mode m) {
  switch (m) {
    case MODE_OFFICE: return "OFFICE";
    case MODE_LOCKED: return "LOCKED";
    case MODE_FREE: return "FREE";
    default: return "UNKNOWN";
  }
}

/**
 * @brief Signals that access has been granted.
 *
 * Turns on the green LED and sounds a short high-pitched buzzer tone.
 * Uses vTaskDelay so the delay relents to the RTOS scheduler instead of
 * blocking the processor with a wait that's busy.
 *
 * @param None
 * @return None
 */
void grantAccess() {
  digitalWrite(RED_LED, LOW);
  digitalWrite(ACCESS_LED, HIGH);
  tone(BUZZER, 1000);
  vTaskDelay(pdMS_TO_TICKS(200));
  noTone(BUZZER);
  digitalWrite(ACCESS_LED, LOW);
}

/**
 * @brief Indicates that access is denied.
 *
 * Red LED is turned on and a low-pitched tone is played on the buzzer.
 * vTaskDelay used so the delay relents to the RTOS scheduler instead of
 * blocking the processor with a wait that's busy.
 *
 * @param None
 * @return None
 */
void denyAccess() {
  digitalWrite(ACCESS_LED, LOW);
  digitalWrite(RED_LED, HIGH);
  tone(BUZZER, 400);
  vTaskDelay(pdMS_TO_TICKS(300));
  noTone(BUZZER);
  digitalWrite(RED_LED, LOW);
}

/* ==================== TASKS ==================== */

/**
 * @brief The IR receiver is polled and the current operating mode is updated.
 *
 * Runs on Core 0 at around 64 Hz. When a valid IR command is decoded,
 * the command byte is matched to one of the three operating modes and the
 * currentMode is updated under modeMutex. Repeat frames are ignored.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void IRTask(void *p) {
  IrReceiver.begin(IR_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("IR receiver ready");
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    if (IrReceiver.decode()) {
      uint8_t cmd = IrReceiver.decodedIRData.command;
      if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
        Mode newMode = currentMode;
        bool changed = false;
        if (cmd == IR_BTN_OFFICE) {
          newMode = MODE_OFFICE;
          changed = true;
        }
        else if (cmd == IR_BTN_LOCKED) {
          newMode = MODE_LOCKED;
          changed = true;
        }
        else if (cmd == IR_BTN_FREE) {
          newMode = MODE_FREE;
          changed = true;
        }
        else {
          Serial.print("IR: unknown command 0x");
          Serial.println(cmd, HEX);
        }

        if (changed) {
          if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
            currentMode = newMode;
            xSemaphoreGive(modeMutex);
          }
          Serial.print("IR: mode changed to ");
          Serial.println(modeName(newMode));
        }
      }
      IrReceiver.resume();
    }
    vTaskDelayUntil(&last, IR_PERIOD);
  }
}

/**
 * @brief Checks the RFID reader at regular intervals and sends RFID events to ControlTask.
 * Runs at 50 Hz on Core 0. Once a new card is scanned, the task reads the UID,
 * compares it with the authorized UID, and sends an EventMsg to eventQueue.
 * A cooldown avoids repeated triggering from the same card tap.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void RFIDTask(void *p) {
  TickType_t last = xTaskGetTickCount();
  TickType_t cooldownUntil = 0;
  for (;;) {
    TickType_t now = xTaskGetTickCount();
    if (!rfidReady && now >= cooldownUntil) {
      rfidReady = true;
      Serial.println("RFID ready");
    }

    if (rfidReady && rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
      Serial.print("Scanned UID: ");
      for (byte i = 0; i < rfid.uid.size; i++) {
        if (rfid.uid.uidByte[i] < 0x10) Serial.print("0");
        Serial.print(rfid.uid.uidByte[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      rfidReady = false;
      cooldownUntil = now + RFID_COOLDOWN_TICKS;
      EventMsg msg;
      msg.type  = EVT_RFID;
      msg.uidOk = uidMatch(rfid.uid.uidByte, rfid.uid.size);
      xQueueSend(eventQueue, &msg, portMAX_DELAY);
      rfid.PICC_HaltA();
      rfid.PCD_StopCrypto1();
    }
    vTaskDelayUntil(&last, RFID_PERIOD);
  }
}

/**
 * @brief Reads the present time from the RTC and updates the shared time state.
 *
 * Runs on Core 1 once every second. The current RTC hour and minute are stored
 * in nowHour and nowMinute under protection of timeMutex.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void RTCTask(void *p) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    DateTime t = rtc.now();
    if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
      nowHour   = t.hour();
      nowMinute = t.minute();
      xSemaphoreGive(timeMutex);
    }
    vTaskDelayUntil(&last, RTC_PERIOD);
  }
}

/**
 * @brief Main decision making task
 *
 * Blocks on eventQueue waiting for RFID events. For each event, it reads the
 * present mode and present time, applies the access policy, then sends the
 * result to both the UI and LCD queues.
 *
 * Access logic:
 * - MODE_FREE: always grant
 * - MODE_LOCKED: always deny
 * - MODE_OFFICE: grant for authorized UID and if present time is in office hours.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void ControlTask(void *p) {
  for (;;) {
    EventMsg msg;
    if (xQueueReceive(eventQueue, &msg, portMAX_DELAY) == pdTRUE) {
      if (msg.type == EVT_RFID) {
        int h = 0;
        int m = 0;
        if (xSemaphoreTake(timeMutex, portMAX_DELAY) == pdTRUE) {
          h = nowHour;
          m = nowMinute;
          xSemaphoreGive(timeMutex);
        }

        Mode mode = MODE_OFFICE;
        if (xSemaphoreTake(modeMutex, portMAX_DELAY) == pdTRUE) {
          mode = currentMode;
          xSemaphoreGive(modeMutex);
        }

        bool allow = false;
        if (mode == MODE_FREE) allow = true;
        else if (mode == MODE_LOCKED) allow = false;
        else allow = msg.uidOk && officeHours(h);

        UiMsg u;
        u.action = allow ? UI_GRANT : UI_DENY;
        xQueueSend(uiQueue, &u, portMAX_DELAY);
        xQueueSend(lcdQueue, &u, 0);
        Serial.print("RFID uidOk=");
        Serial.print(msg.uidOk ? "true" : "false");
        Serial.print(" time=");
        Serial.print(h);
        Serial.print(":");
        if (m < 10) Serial.print("0");
        Serial.print(m);
        Serial.print(" mode=");
        Serial.print(modeName(mode));
        Serial.print(" -> ");
        Serial.println(allow ? "GRANT" : "DENY");
      }
    }
  }
}

/**
 * @brief LEDs and buzzer are driven based on the access.
 *
 * Runs at 50 Hz on Core 1. Checks uiQueue for pending UI actions and does
 * the grant or deny feedback routine correspondingly.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void UITask(void *p) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    UiMsg msg;

    if (xQueueReceive(uiQueue, &msg, 0) == pdTRUE) {
      if (msg.action == UI_GRANT) grantAccess();
      else denyAccess();
    }
    vTaskDelayUntil(&last, UI_PERIOD);
  }
}

/**
 * @brief Manages the 16x2 I2C LCD display.
 *
 * Runs on Core 1 at 10 Hz. Row 0 displays the current system mode and is only
 * redrawn when the mode changes. Row 1 displays "RFID ready" during idle,
 * switches to "GRANT" or "DENY" when a scan result arrives, and reverts back
 * to "RFID ready" when the RFID cooldown ends.
 *
 * @param p Unused FreeRTOS task parameter.
 * @return None. This task runs forever.
 */
void LCDTask(void *p) {
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Mode: OFFICE    ");
  lcd.setCursor(0, 1);
  lcd.print("RFID ready      ");

  Mode lastMode = MODE_OFFICE;
  bool showingResult = false;

  TickType_t last = xTaskGetTickCount();

  for (;;) {
    Mode mode = MODE_OFFICE;
    if (xSemaphoreTake(modeMutex, 0) == pdTRUE) {
      mode = currentMode;
      xSemaphoreGive(modeMutex);
    }

    if (mode != lastMode)
    {
      lastMode = mode;
      char buf[17];
      snprintf(buf, sizeof(buf), "Mode: %-10s", modeName(mode));
      lcd.setCursor(0, 0);
      lcd.print(buf);
    }

    UiMsg msg;
    if (xQueueReceive(lcdQueue, &msg, 0) == pdTRUE) {
      showingResult = true;
      lcd.setCursor(0, 1);
      lcd.print(msg.action == UI_GRANT
                  ? "GRANT           "
                  : "DENY            ");
    }

    if (showingResult && rfidReady) {
      showingResult = false;
      lcd.setCursor(0, 1);
      lcd.print("RFID ready      ");
    }

    vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
  }
}

/* ==================== SETUP ==================== */

/**
 * @brief Standard Arduino setup function.
 *
 * Serial, GPIO outputs, SPI, RFID reader, I2C bus, RTC, RTOS
 * queues and mutexes are initialized, watchdog is disabled, and all FreeRTOS tasks are created.
 */
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(ACCESS_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(ACCESS_LED, LOW);
  digitalWrite(RED_LED, LOW);
  noTone(BUZZER);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, RFID_SS);
  rfid.PCD_Init();
  Serial.println("RFID ready");
  Wire.begin(SDA_PIN, SCL_PIN);

  if (rtc.begin()) {
    Serial.println("RTC ready");
    DateTime t = rtc.now();
    nowHour = t.hour();
    nowMinute = t.minute();

    Serial.print("Current time: ");
    Serial.print(nowHour);
    Serial.print(":");
    if (nowMinute < 10) Serial.print("0");
    Serial.println(nowMinute);
  }
  else {
    Serial.println("RTC not detected");
  }

  eventQueue = xQueueCreate(10, sizeof(EventMsg));
  uiQueue = xQueueCreate(10, sizeof(UiMsg));
  lcdQueue = xQueueCreate(5, sizeof(UiMsg));
  timeMutex = xSemaphoreCreateMutex();
  modeMutex = xSemaphoreCreateMutex();

  if (eventQueue == NULL || uiQueue == NULL || lcdQueue == NULL ||
      timeMutex == NULL  || modeMutex == NULL) {
    Serial.println("RTOS allocation failed");
    while (1) { 
      delay(1000); 
      }
  }

  esp_task_wdt_deinit();

  xTaskCreatePinnedToCore(RFIDTask, "RFID", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(IRTask, "IR", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(RTCTask, "RTC", 3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(ControlTask, "CTRL", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(UITask, "UI", 3072, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(LCDTask,"LCD", 3072, NULL, 1, NULL, 1);

  Serial.println("System ready");
  Serial.println("Default mode: Office hours");
  Serial.println("IR remote: btn1=OFFICE  btn2=LOCKED  btn3=FREE");
}

/* ==================== LOOP ==================== */

/**
 * @brief Standard Arduino loop function.
 *
 * All logic in FreeRTOS tasks, so loop() is empty.
 */
void loop()
{
}