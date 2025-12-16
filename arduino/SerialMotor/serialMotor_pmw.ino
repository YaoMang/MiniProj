// ===== Timer1 : OC1A = D9 (CTC toggle) =====
void timer1_init() {
    pinMode(9, OUTPUT);

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;

    // CTC mode
    TCCR1B |= (1 << WGM12);

    // ❌ 不要在 init 里连接 OC1A
    // ❌ 不要在 init 里启动定时器

    OCR1A = 999;   // 先给个默认值
    digitalWrite(9, LOW);
}


void timer1_set_freq(uint32_t hz) {
    if (hz < 1) hz = 1;

    uint32_t top = F_CPU / (2UL * hz) - 1;
    if (top > 65535) top = 65535;
    if (top < 1) top = 1;

    OCR1A = (uint16_t)top;

    // 确保 OC1A 连接
    TCCR1A |= (1 << COM1A0);

    // ⭐关键：确保 Timer1 在跑（stop 过必须重启）
    TCCR1B = (TCCR1B & ~((1<<CS12)|(1<<CS11)|(1<<CS10)))
           | (1<<CS10);
}

void timer1_stop() {
    // 停止计数器
    TCCR1B &= ~((1<<CS12)|(1<<CS11)|(1<<CS10));

    // 断开 OC1A
    TCCR1A &= ~(1 << COM1A0);

    digitalWrite(9, LOW);
}

// ===== Timer2 : OC2A = D11 (CTC toggle, 50% 方波) =====
void timer2_init() {
    pinMode(11, OUTPUT);

    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;

    // CTC 模式
    TCCR2A |= (1 << WGM21);

    // Toggle OC2A on compare match → 50% 方波
    //TCCR2A |= (1 << COM2A0);

    // 默认 prescaler = 8
    //TCCR2B |= (1 << CS21);

    // 默认 ≈ 8kHz
    OCR2A = 124; // 16MHz / (2 * 8 * (124+1)) ≈ 8000 Hz
}

void timer2_set_freq(uint32_t hz) {
    if (hz < 1) hz = 1;

    const uint16_t presc[] = {1, 8, 32, 64, 128, 256, 1024};
    const uint8_t  bits[]  = {
        (1<<CS20),
        (1<<CS21),
        (1<<CS21)|(1<<CS20),
        (1<<CS22),
        (1<<CS22)|(1<<CS20),
        (1<<CS22)|(1<<CS21),
        (1<<CS22)|(1<<CS21)|(1<<CS20)
    };

    uint32_t bestErr = 0xFFFFFFFF;
    uint8_t bestIdx = 1;

    for (uint8_t i = 0; i < 7; i++) {
        uint32_t ocr = F_CPU / (2UL * presc[i] * hz);
        if (ocr == 0) continue;
        ocr -= 1;
        if (ocr > 255) continue;

        uint32_t realHz = F_CPU / (2UL * presc[i] * (ocr + 1));
        uint32_t err = (realHz > hz) ? (realHz - hz) : (hz - realHz);

        if (err < bestErr) {
            bestErr = err;
            bestIdx = i;
            OCR2A = (uint8_t)ocr;
        }
    }

    // ✅ 显式启用 OC2A toggle（关键）
    TCCR2A |= (1 << COM2A0);

    // ✅ 启动 Timer2
    TCCR2B = (TCCR2B & ~((1<<CS22)|(1<<CS21)|(1<<CS20)))
           | bits[bestIdx];
}

void timer2_stop() {
    TCCR2A &= ~(1 << COM2A0);
    digitalWrite(11, LOW);
}

class DM542 {
public:
    int pulPin, dirPin, enaPin;
    bool dirPolarity;
    bool enaPolarity;

    unsigned long stepIntervalUs;
    unsigned long lastStepTime;
    unsigned long stopTime;
    bool running;

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

    // ===== BF：时间模式（硬件脉冲）=====
    void start(bool direction, float speedHz, uint32_t durationMs) {
        pulseMode = false;
        pulseCounter = 0;

        digitalWrite(dirPin, direction ? dirPolarity : !dirPolarity);

        uint32_t hz = (speedHz <= 0) ? 1 : (uint32_t)speedHz;

        if (pulPin == 9) {
            timer1_set_freq(hz);
        } else if (pulPin == 11) {
            timer2_set_freq(hz);
        }

        stopTime = micros() + (unsigned long)durationMs * 1000UL;
        running = true;
    }

    // ===== AF：脉冲数模式（保持原软件逻辑）=====
    void startByPulse(bool direction, float speedHz, uint32_t pulses) {
        pulseMode = true;
        pulseCounter = 0;
        targetPulses = pulses;

        digitalWrite(dirPin, direction ? dirPolarity : !dirPolarity);

        if (speedHz <= 0) speedHz = 1;
        stepIntervalUs = (unsigned long)(1000000.0 / speedHz);

        running = true;
        lastStepTime = micros();
    }

    void update() {
        if (!running) return;

        unsigned long now = micros();

        // ===== BF：到时间就停 =====
        if (!pulseMode) {
            if ((long)(now - stopTime) >= 0) {
                if (pulPin == 9) timer1_stop();
                else if (pulPin == 11) timer2_stop();
                running = false;
            }
            return;
        }

        // ===== AF：原样 software step =====
        if ((long)(now - lastStepTime) >= (long)stepIntervalUs) {
            lastStepTime = now;

            digitalWrite(pulPin, HIGH);
            delayMicroseconds(15);
            digitalWrite(pulPin, LOW);
            delayMicroseconds(15);

            pulseCounter++;
            if (pulseCounter >= targetPulses) {
                running = false;
            }
        }
    }
};

#define NUM_MOTORS 2

DM542 motors[NUM_MOTORS] = {
  DM542(9, 7),   // Motor 0
  DM542(11, 2)    // Motor 1
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

void setup() {
    Serial.begin(115200);

    timer1_init();
    timer2_init();

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].begin();
    }

    Serial.println("READY");
}

void loop() {
    readSerial();   // 1. 处理串口协议（可能启动新任务）

    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].update();   // 2. 仅做“是否该停”的状态检查
    }
}
  