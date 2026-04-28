#include "AnglePID.h"
#include "SpeedPID.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>

constexpr int8_t PIN_STEER_MOTOR_PWM = 22;
constexpr int8_t PIN_STEER_MOTOR_DIR = 23;
constexpr int8_t PIN_ENC_A           = 27;
constexpr int8_t PIN_ENC_B           = 14;
constexpr int8_t PIN_ENC_Z           = 25;

constexpr int8_t W1_CH = 0;

constexpr double ENC_RESOLUTION                  = 4096.;
constexpr double STEER_GEAR_RATIO_MOTOR_TO_STEER = 2.0; // 1:2（ステア角に対しモータ2回転）

constexpr int16_t PWM_LIMIT    = 255;
constexpr int     PWM_DEADBAND = 8;

constexpr int      HOMING_PWM        = 70;
constexpr uint32_t HOMING_TIMEOUT_MS = 8000;

// グローバル変数
ESP32Encoder encoder;
AnglePID     anglePID(1.5, 0.0, 0.01, -300.0, 300.0, ENC_RESOLUTION, -10.0, 10.0);
SpeedPID     speed_pid_1(3., 3., 0.00, -PWM_LIMIT, PWM_LIMIT);

volatile long encoderCount      = 0;
volatile bool zeroPointDetected = false;

bool homingDone  = false;
bool homingError = false;

// Z相割り込みハンドラ
void IRAM_ATTR onZPhase() {
    encoderCount      = 0;
    zeroPointDetected = true;
}

// signは、モータの配線や取り付け向きの補正
void setMotor(int8_t dirPin, int pwmCh, int sign, int pwm_signed) {
    int duty = sign * pwm_signed;
    duty     = constrain(duty, -PWM_LIMIT, PWM_LIMIT);

    if (abs(duty) < PWM_DEADBAND) {
        ledcWrite(pwmCh, 0);
        return;
    }
    digitalWrite(dirPin, (duty > 0) ? HIGH : LOW);
    ledcWrite(pwmCh, abs(duty));
}

static inline long steerDegToEncCount(double steer_deg) {
    // ステア角[deg] -> エンコーダカウント（ギア比考慮）
    return lroundf((steer_deg * STEER_GEAR_RATIO_MOTOR_TO_STEER * ENC_RESOLUTION) / 360.0);
}

void stopSteerMotor() {
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, 0);
}

bool runSteerHoming() {
    // 1) ロリコン(エンコーダ)をリセットし現在位置を0
    noInterrupts();
    zeroPointDetected = false;
    interrupts();
    encoder.clearCount();
    const long plus180  = steerDegToEncCount(+180.0); // +180°(ステア)
    const long minus180 = steerDegToEncCount(-180.0); // -180°(ステア)

    // 2) +180°回転開始
    uint32_t t0 = millis();
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, +HOMING_PWM);

    while (encoder.getCount() < plus180) {
        if (zeroPointDetected) {
            // 3) 途中でZ相検出 -> リセットして停止
            stopSteerMotor();
            encoder.clearCount();
            noInterrupts();
            zeroPointDetected = false;
            interrupts();
            Serial.println("[HOMING] Z detected on + sweep. Homing OK.");
            return true;
        }
        if (millis() - t0 > HOMING_TIMEOUT_MS) break;
        delay(2);
    }

    // +180までで未検出 -> 4) 逆回転して -180 まで（合計 -360）
    t0 = millis();
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, -HOMING_PWM);

    while (encoder.getCount() > minus180) {
        if (zeroPointDetected) {
            // 途中でZ相検出 -> リセットして停止
            stopSteerMotor();
            encoder.clearCount();
            noInterrupts();
            zeroPointDetected = false;
            interrupts();
            Serial.println("[HOMING] Z detected on - sweep. Homing OK.");
            return true;
        }
        if (millis() - t0 > HOMING_TIMEOUT_MS) break;
        delay(2);
    }

    // 5) -180まで行っても未検出 -> 回路エラー
    stopSteerMotor();
    Serial.println("[HOMING][ERROR] Z phase not detected. Check encoder/Z wiring.");
    return false;
}

void setup() {
    Serial.begin(115200);

    // モーターピン初期化
    pinMode(PIN_STEER_MOTOR_DIR, OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachFullQuad(PIN_ENC_A, PIN_ENC_B);
    pinMode(PIN_ENC_Z, INPUT);
    encoder.clearCount();
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_Z), onZPhase, RISING);

    ledcSetup(W1_CH, 12800, 8);
    ledcAttachPin(PIN_STEER_MOTOR_PWM, W1_CH);

    PS4.begin("00:11:22:33:44:55");

    homingDone  = runSteerHoming();
    homingError = !homingDone;

    Serial.println("Setup complete");
}

unsigned long     last          = micros();
constexpr int32_t CONTROL_CYCLE = 5000;

void loop() {
    if (homingError) {
        static uint32_t last = 0;
        if (millis() - last > 1000) {
            last = millis();
            Serial.println("[HOMING ERROR] paused. power-cycle after fixing wiring.");
        }
        stopSteerMotor();
        delay(10);
        return;
    }

    if (!PS4.isConnected()) {
        stopSteerMotor();
        delay(10);
        return;
    }

    unsigned long now = micros();
    if (now - last < CONTROL_CYCLE) return;

    double dt = (now - last) * 1.e-6;
    last      = now;

    // 右スティックをベクトルとして読む
    int rx = PS4.RStickX(); // -128..127
    int ry = PS4.RStickY();

    // 角度 [deg]
    double stickAngleDeg = atan2((double)ry, (double)rx) * 180.0 / M_PI;

    // 距離（0..約180）
    double stickMag = hypot((double)rx, (double)ry);

    // デッドゾーン
    const double DEADZONE   = 15.0;
    double       moveOutput = 0.0;
    if (stickMag > DEADZONE) {
        moveOutput = (stickMag - DEADZONE) / (127.0 - DEADZONE) * PWM_LIMIT;
        moveOutput = constrain(moveOutput, 0.0, (double)PWM_LIMIT);
    }

    // ステア角目標として使うなら
    double angleTargetDeg = stickAngleDeg;

    double currentAngleDeg = (encoder.getCount() * 360.0 / ENC_RESOLUTION) / STEER_GEAR_RATIO_MOTOR_TO_STEER;

    int steerOutput = (int)anglePID.update(angleTargetDeg, currentAngleDeg, dt);
    setMotor(PIN_STEER_MOTOR_DIR, W1_CH, +1, steerOutput);

    Serial.printf("rx=%d ry=%d angle=%.1f mag=%.1f move=%.1f\n", rx, ry, stickAngleDeg, stickMag, moveOutput);

    delay(10);
}
