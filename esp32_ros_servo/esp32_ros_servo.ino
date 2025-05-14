#include <Arduino.h>
#include <ESP32Servo.h>

// Debug flag
#define DEBUG false

// Protocol constants
#define START_BYTE 0xAA
#define END_BYTE 0x55
#define CMD_SET_ANGLE 0x01
#define CMD_HEARTBEAT 0x02
#define RESP_STATE 0x03
#define RESP_ERROR 0x04

// Error codes
#define ERR_NONE 0x00
#define ERR_INVALID_ANGLE 0x01
#define ERR_SERVO_FAILURE 0x02

// Ring buffer size
#define RING_BUFFER_SIZE 32

Servo myservo;
int currentAngle = 0;
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 100; // 100 ms
unsigned long lastStateUpdate = 0;
unsigned long lastLoopTime = 0;
unsigned long loopTime = 0;

// Ring buffer for incoming messages
struct RingBuffer {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
};

RingBuffer rxBuffer = {0};

// Simple message structure
struct SimpleMessage {
    uint8_t start;
    uint8_t type;
    uint8_t data;
    uint8_t end;
};

// Function to add byte to ring buffer
void addToBuffer(uint8_t byte) {
    if (rxBuffer.count < RING_BUFFER_SIZE) {
        rxBuffer.buffer[rxBuffer.head] = byte;
        rxBuffer.head = (rxBuffer.head + 1) % RING_BUFFER_SIZE;
        rxBuffer.count++;
    }
}

// Function to get byte from ring buffer
bool getFromBuffer(uint8_t* byte) {
    if (rxBuffer.count > 0) {
        *byte = rxBuffer.buffer[rxBuffer.tail];
        rxBuffer.tail = (rxBuffer.tail + 1) % RING_BUFFER_SIZE;
        rxBuffer.count--;
        return true;
    }
    return false;
}

void sendMessage(uint8_t type, uint8_t data) {
    SimpleMessage msg;
    msg.start = START_BYTE;
    msg.type = type;
    msg.data = data;
    msg.end = END_BYTE;
    
    // Debug print
    if (DEBUG) {
        Serial.print("Sending - Type: 0x");
        Serial.print(type, HEX);
        Serial.print(", Data: 0x");
        Serial.println(data, HEX);
    }
    
    // Send message
    Serial.write((uint8_t*)&msg, sizeof(SimpleMessage));
    Serial.flush(); // Wait for transmission to complete
}

void sendError(uint8_t errorCode) {
    sendMessage(RESP_ERROR, errorCode);
}

bool handleSetAngle(uint8_t angle) {
    if (angle > 180) {
        sendError(ERR_INVALID_ANGLE);
        return false;
    }
    
    // Try to write to servo
    if (!myservo.attached()) {
        sendError(ERR_SERVO_FAILURE);
        return false;
    }
    
    myservo.write(angle);
    currentAngle = angle;
    
    // Send acknowledgment
    sendMessage(RESP_STATE, angle);
    return true;
}

void sendState() {
    sendMessage(RESP_STATE, currentAngle);
}

void sendHeartbeat() {
    sendMessage(CMD_HEARTBEAT, 0);
}

void setup() {
    Serial.begin(115200);
    // Wait for serial to be ready
    while (!Serial) {
        delay(10);
    }
    if (DEBUG) {
        Serial.println("ESP32 Servo Bridge Started");
    }
    
    // Initialize servo
    if (!myservo.attach(14)) {  // change to your actual servo pin
        if (DEBUG) {
            Serial.println("Failed to attach servo!");
        }
    }
    currentAngle = myservo.read();
}

void loop() {
    unsigned long loopStart = millis();
    
    // Read all available bytes into ring buffer
    while (Serial.available()) {
        addToBuffer(Serial.read());
    }
    
    // Process messages from ring buffer
    while (rxBuffer.count >= sizeof(SimpleMessage)) {
        SimpleMessage msg;
        bool validMessage = true;
        
        // Read message from buffer
        for (int i = 0; i < sizeof(SimpleMessage); i++) {
            uint8_t byte;
            if (!getFromBuffer(&byte)) {
                validMessage = false;
                break;
            }
            ((uint8_t*)&msg)[i] = byte;
        }
        
        if (validMessage) {
            // Debug print received message
            if (DEBUG) {
                Serial.print("Received - Start: 0x");
                Serial.print(msg.start, HEX);
                Serial.print(", Type: 0x");
                Serial.print(msg.type, HEX);
                Serial.print(", Data: 0x");
                Serial.print(msg.data, HEX);
                Serial.print(", End: 0x");
                Serial.println(msg.end, HEX);
            }
            
            // Validate message
            if (msg.start == START_BYTE && msg.end == END_BYTE) {
                // Process message
                switch (msg.type) {
                    case CMD_SET_ANGLE:
                        handleSetAngle(msg.data);
                        break;
                    case CMD_HEARTBEAT:
                        // Just acknowledge heartbeat
                        sendHeartbeat();
                        break;
                }
            } else if (DEBUG) {
                Serial.println("Invalid message format");
            }
        }
    }

    // Send periodic updates
    unsigned long currentTime = millis();
    
    // Send heartbeat
    if (currentTime - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = currentTime;
    }

    // Calculate and print loop time
    loopTime = millis() - loopStart;
    if (DEBUG) {
        Serial.print("Loop time: ");
        Serial.print(loopTime);
        Serial.println(" ms");
    }
}
