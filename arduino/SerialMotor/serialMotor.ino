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

    // NEW: Pulse mode
    bool pulseMode = false;
    uint32_t targetPulses = 0;
    uint32_t pulseCounter = 0;

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

    // =========================
    // 原始 BF 指令：按时间运动 (durationMs)
    // =========================
    void start(bool direction, float speedHz, uint32_t durationMs) {
        pulseMode = false;    // NEW
        pulseCounter = 0;     // NEW

        digitalWrite(dirPin, direction ? dirPolarity : !dirPolarity);

        if (speedHz <= 0) speedHz = 1;
        stepIntervalUs = (unsigned long)(1000000.0 / speedHz);

        running = true;
        lastStepTime = micros();

        unsigned long durationUs = (unsigned long)durationMs * 1000UL;
        stopTime = lastStepTime + durationUs;
    }

    // =========================
    // 新增 AF 指令：按脉冲数运动 (pulseNum)
    // =========================
    void startByPulse(bool direction, float speedHz, uint32_t pulses) {
        pulseMode = true;        // NEW
        pulseCounter = 0;        // NEW
        targetPulses = pulses;   // NEW

        digitalWrite(dirPin, direction ? dirPolarity : !dirPolarity);

        if (speedHz <= 0) speedHz = 1;
        stepIntervalUs = (unsigned long)(1000000.0 / speedHz);

        running = true;
        lastStepTime = micros();
    }

    // =========================
    // 更新（同时支持两种模式）
    // =========================
    void update() {
        if (!running) return;

        unsigned long now = micros();

        // 时间模式超时判定
        if (!pulseMode) {
            if ((long)(now - stopTime) >= 0) {
                running = false;
                return;
            }
        }

        // 是否到了发步进脉冲的时间
        if ((long)(now - lastStepTime) >= (long)stepIntervalUs) {
            lastStepTime = now;

            // DM542 脉宽
            digitalWrite(pulPin, HIGH);
            delayMicroseconds(15);
            digitalWrite(pulPin, LOW);
            delayMicroseconds(15);

            // 脉冲模式计数
            if (pulseMode) {
                pulseCounter++;
                if (pulseCounter >= targetPulses) {
                    running = false;
                }
            }
        }
    }
};


// =========================
// SERIAL COMMAND PROTOCOL
// =========================
#define FRAME_HEADER_TIME 0xBF
#define FRAME_HEADER_PULSE 0xAF
#define FRAME_LENGTH 11

struct CommandFrame {
    uint8_t header;
    uint8_t motorMask;
    uint8_t directionMask;
    int32_t speedHz;      // BF: speed | AF: speed
    int32_t durationMs;   // BF: duration | AF: pulseNum
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
            if (byte == FRAME_HEADER_TIME || byte == FRAME_HEADER_PULSE) {
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
    memcpy(&frame, buffer, FRAME_LENGTH);

    // 检查指令类别
    bool isTimeMode  = (frame.header == FRAME_HEADER_TIME);
    bool isPulseMode = (frame.header == FRAME_HEADER_PULSE);
    if (!isTimeMode && !isPulseMode) return;

    // 参数防御
    if (frame.durationMs < 0) frame.durationMs = -frame.durationMs;
    if (frame.speedHz < 0) frame.speedHz = -frame.speedHz;

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (frame.motorMask & (1 << i)) {
            bool direction = (frame.directionMask & (1 << i)) != 0;

            if (isTimeMode) {
                // BF 模式：按时间
                motors[i].start(direction,
                                (float)frame.speedHz,
                                (uint32_t)frame.durationMs);
            } else {
                // AF 模式：按脉冲
                motors[i].startByPulse(direction,
                                       (float)frame.speedHz,
                                       (uint32_t)frame.durationMs);
            }
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
