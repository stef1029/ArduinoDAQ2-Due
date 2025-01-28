#include <Arduino.h>
#include <stdint.h>

// Same pin definitions
#define BUZZER1 41
#define BUZZER2 37
#define BUZZER3 33
#define BUZZER4 30
#define BUZZER5 34
#define BUZZER6 38
#define LED_1   39
#define LED_2   35
#define LED_3   31
#define LED_4   32
#define LED_5   36
#define LED_6   40
#define VALVE1  47
#define VALVE2  45
#define VALVE3  43
#define VALVE4  42
#define VALVE5  44
#define VALVE6  46
#define SENSOR1 25
#define SENSOR2 27
#define SENSOR3 29
#define SENSOR4 28
#define SENSOR5 26
#define SENSOR6 24
#define SPOT1   12
#define SPOT2   7
#define SPOT3   8
#define SPOT4   9
#define SPOT5   10
#define SPOT6   11
#define IR      6
#define GO_CUE  48
#define NOGO_CUE 50
#define SYNC    2
#define SYNC_GND 3

#define NEW_PIN1 49
#define NEW_PIN2 51
#define NEW_PIN3 62 // A8 pin

// Pin list
uint8_t pin_list[] = {
  7, 8, 9, 10, 11, 12,
  24, 25, 26, 27, 28, 29,
  30, 31, 32, 33, 34, 35,
  36, 37, 38, 39, 40, 41,
  42, 43, 44, 45, 46, 47,
  48, 50, 62, 63
};

const unsigned int num_pins = sizeof(pin_list) / sizeof(pin_list[0]);

// Each message is 11 bytes
static const int MESSAGE_SIZE = 11;

// We'll collect multiple messages before sending them to the host
static const int CHUNK_SIZE = 100;  // Adjust to your preference

static byte chunkBuffer[CHUNK_SIZE * MESSAGE_SIZE];
static int chunkIndex = 0; // how many messages in the current batch

unsigned long message_count = 0;
bool Send_messages = true;
bool start_wait = true;

// User-configurable delay in milliseconds between reads/messages
// If set to 0, there is effectively no delay.
int samplingDelay = 0;  

//
// Build one 11-byte message in 'dest':
//  - Start byte (0x01)
//  - Interleaved message_number (4 bytes) + message (5 bytes)
//  - End byte (0x02)
//
void buildMessage(uint64_t message, unsigned long msgNum, byte* dest) {
  dest[0] = 0x01; // Start byte

  dest[1]  = (msgNum >> 24) & 0xFF;     
  dest[2]  = (message >> 32) & 0xFF;   
  dest[3]  = (msgNum >> 16) & 0xFF;    
  dest[4]  = (message >> 24) & 0xFF;   
  dest[5]  = (msgNum >>  8) & 0xFF;    
  dest[6]  = (message >> 16) & 0xFF;   
  dest[7]  =  msgNum        & 0xFF;    
  dest[8]  = (message >>  8) & 0xFF;   
  dest[9]  =  message       & 0xFF;    

  dest[10] = 0x02; // End byte
}

//
// Send the entire chunkBuffer as one big USB transfer.
// In this version, we do NOT toggle SYNC here; 
// we do a SYNC pulse for every read instead.
//
void flushChunk() {
  if (chunkIndex == 0) {
    return; // no data to send
  }

  // Send all data in one big write
  SerialUSB.write(chunkBuffer, chunkIndex * MESSAGE_SIZE);

  // Reset index
  chunkIndex = 0;
}

void setup() {
  for (unsigned int i = 0; i < num_pins; i++) {
    pinMode(pin_list[i], INPUT);
    digitalWrite(pin_list[i], LOW);
  }

  pinMode(13, OUTPUT);
  pinMode(SYNC, OUTPUT);
  pinMode(SYNC_GND, OUTPUT);
  digitalWrite(SYNC_GND, LOW);
  

  SerialUSB.begin(115200);

  // (Optional) Set an initial delay here, for example:
  // samplingDelay = 5; // 5 ms delay
}

uint64_t previous_state = 0;

void resetDue() {
  RSTC->RSTC_CR = RSTC_CR_PROCRST | RSTC_CR_PERRST | RSTC_CR_EXTRST | (0xA5 << 24);
}

void resetUSB() {
  // End USB connection
  SerialUSB.end();
  delay(1000);

  // Trigger USB reset via hardware register (force re-enumeration)
  *((uint32_t*)0x400E0E00U) = 0x80; // Disable USB
  delay(1000);
  *((uint32_t*)0x400E0E00U) = 0x00; // Enable USB

  SerialUSB.begin(115200);  // Restart USB communication
}

void loop() {
  // Wait for 's' from the host to start
  while (start_wait) {
    if (SerialUSB.available() > 0) {
      if (SerialUSB.read() == 's') {
        SerialUSB.print("s");
        start_wait = false;
        digitalWrite(13, HIGH);
        // Initialize previous_state with first reading
        previous_state = 0;
        for (unsigned int i = 0; i < num_pins; i++) {
          if (digitalRead(pin_list[i]) == HIGH) {
            previous_state |= ((uint64_t)1 << i);
          }
        }
      }
    }
  }

  // Main loop: read pins and send only on change
  while (Send_messages) {
    uint64_t current_state = 0;

    // --- 1) Toggle SYNC for each reading ------------------------------------
    digitalWrite(SYNC, HIGH);
    
    // --- 2) Gather current pin states --------------------------------------
    for (unsigned int i = 0; i < num_pins; i++) {
      if (digitalRead(pin_list[i]) == HIGH) {
        current_state |= ((uint64_t)1 << i);
      }
    }
    digitalWrite(SYNC, LOW);

    // --- 3) Check if state changed ----------------------------------------
    if (current_state != previous_state) {
      // State changed - send a message
      byte message_buffer[MESSAGE_SIZE];
      buildMessage(current_state, message_count, message_buffer);
      SerialUSB.write(message_buffer, MESSAGE_SIZE);
      
      // Update previous state
      previous_state = current_state;
    }

    // Increment message count for every read, not just when we send
    message_count++;

    // --- 4) Check for 'e' from the host to end ----------------------------
    if (SerialUSB.available() > 0) {
      if (SerialUSB.read() == 'e') {
        start_wait = true;
        SerialUSB.flush();
        // SerialUSB.end();
        // delay(1000);
        // SerialUSB.begin(115200);
        resetUSB();
        digitalWrite(13, LOW);
        message_count = 0;
        break;
      }
    }

    // --- 5) Apply the user-configurable delay (if any) --------------------
    if (samplingDelay > 0) {
      delay(samplingDelay);
    }
  }
}