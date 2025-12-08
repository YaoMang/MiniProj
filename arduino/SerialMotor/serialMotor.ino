// =========================
// DM542 Driver Class
// =========================
class DM542 {
public:
    int pulPin, dirPin, enaPin;
    bool dirPolarity;
    bool enaPolarity;

    unsigned long stepIntervalUs;    
    unsigned long lastStepTime;      
    unsigned long stopTime;          
    bool running;

    DM542(int pul, int dir, int ena = -1,
          bool dirPol = HIGH, bool enaPol = HIGH)
        : pulPin(pul), dirPin(dir), enaPin(ena),
          dirPolarity(dirPol), enaPolarity(enaPol),
          stepIntervalUs(2000), lastStepTime(0),
          running(false), stopTime(0) {}

    void begin() {
        pinMode(pulPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        if (enaPin != -1) pinMode(enaPin, OUTPUT);
    }

    void enable() {
        if (enaPin != -1) digitalWrite(enaPin, enaPolarity);
    }

    void disable() {
        if (enaPin != -1) digitalWrite(enaPin, !enaPolarity);
    }

    // direction: true/false, speed: Hz, duration: seconds
    void start(bool direction, float speedHz, float durationSec) {
        digitalWrite(dirPin, direction ? dirPolarity : !dirPolarity);

        if (speedHz <= 0) speedHz = 1;  // avoid div by zero
        stepIntervalUs = (unsigned long)(1000000.0 / speedHz);

        running = true;
        lastStepTime = micros();
        unsigned long durationUs = (unsigned long)(durationSec * 1e6);
        stopTime = lastStepTime + durationUs;   // safe (handles micros rollover)
    }

    void stop() {
        running = false;
    }

    void update() {
        if (!running) return;

        unsigned long now = micros();

        // Safe comparison (micros rollover-aware)
        if ((long)(now - stopTime) >= 0) {
            running = false;
            return;
        }

        if ((long)(now - lastStepTime) >= (long)stepIntervalUs) {
            lastStepTime = now;

            // DM542 requires pulse width >= 7.5us
            digitalWrite(pulPin, HIGH);
            delayMicroseconds(50);     // safe pulse width
            digitalWrite(pulPin, LOW);
            delayMicroseconds(50);
        }
    }
};


// =========================
// SERIAL COMMAND PROTOCOL
// =========================
#define FRAME_HEADER 0xBF
#define FRAME_LENGTH 11

struct CommandFrame {
    uint8_t header;
    uint8_t motorMask;
    uint8_t directionMask;
    int32_t speedHz;
    int32_t durationSec;
};

uint8_t buffer[FRAME_LENGTH];
uint8_t index = 0;
bool receiving = false;


// =========================
// MOTOR ARRAY
// =========================
#define NUM_MOTORS 2

DM542 motors[NUM_MOTORS] = {
  DM542(7, 6),   // Motor 0
  DM542(3, 2)    // Motor 1
};


// =========================
// Serial Reader
// =========================
void readSerial() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        if (!receiving) {
            if (byte == FRAME_HEADER) {
                receiving = true;
                index = 0;
                buffer[index++] = byte;
            }
            continue;
        }

        buffer[index++] = byte;

        if (index >= FRAME_LENGTH) {
            receiving = false;
            processFrame();
        }
    }
}


// =========================
// Process Frame
// =========================
void processFrame() {
    CommandFrame frame;
    memcpy(&frame, buffer, FRAME_LENGTH);   // safe, no endian issues

    if (frame.header != FRAME_HEADER) return;

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (frame.motorMask & (1 << i)) {
            bool direction = (frame.directionMask & (1 << i)) != 0;
            motors[i].start(direction, frame.speedHz, frame.durationSec);
        }
    }

    Serial.println("OK");
}


// =========================
// SETUP
// =========================
void setup() {
    Serial.begin(115200);

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].begin();
    }

    Serial.println("READY");
}


// =========================
// LOOP
// =========================
void loop() {
    readSerial();

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].update();
    }
}
