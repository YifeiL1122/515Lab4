/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 * Licensed under the Apache License, Version 2.0
 */

#include <yifeil1122-project-1_inferencing.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// 采样配置
#define SAMPLE_RATE_MS      10                                   // 100 Hz -> 10 ms/样本
#define FEATURE_SIZE        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE  // 模型输入总 float 数
#define SAMPLE_COUNT        (FEATURE_SIZE/3)                     // 每轴样本数
#define CAPTURE_DURATION_MS (SAMPLE_RATE_MS * SAMPLE_COUNT)      // 自动算采集时长

// 硬件映射
#define BUTTON_PIN    3   // 板子 D1 → MCU GPIO3
#define SDA_PIN       6   // 板子 D4 → MCU GPIO6
#define SCL_PIN       7   // 板子 D5 → MCU GPIO7
#define LED_RED_PIN   20  // 板子 D7 → MCU GPIO20
#define LED_GREEN_PIN 8   // 板子 D8 → MCU GPIO8
#define LED_BLUE_PIN  9   // 板子 D9 → MCU GPIO9

// 状态变量
bool    capturing = false;
int     sample_count = 0;
unsigned long last_sample_time   = 0;
unsigned long capture_start_time = 0;

// 结果显示状态
bool    showing_result = false;
unsigned long result_start_time = 0;

float features[FEATURE_SIZE];

// Edge Impulse 数据回调
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, features + offset, length * sizeof(float));
    return 0;
}

void run_inference();
void print_prediction_and_light(const char* label, float confidence);

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        while (1) { delay(10); }
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    Serial.print("MPU6050 initialized. Need ");
    Serial.print(SAMPLE_COUNT);
    Serial.println(" samples (~1 s) per axis.");
    Serial.println("请按下按钮开始采集，按钮优先级最高～");
}

void loop() {
    // ——按钮检测必须最先——
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50);
        if (digitalRead(BUTTON_PIN) == LOW) {
            // 重置所有状态
            capturing = true;
            sample_count = 0;
            capture_start_time = millis();
            last_sample_time = millis();
            showing_result = false;        // 取消结果显示
            // 三色灯全亮提示采集
            digitalWrite(LED_RED_PIN, HIGH);
            digitalWrite(LED_GREEN_PIN, HIGH);
            digitalWrite(LED_BLUE_PIN, HIGH);
            Serial.println("按钮按下：最高优先级开始手势采集…");
            // 等松手
            while (digitalRead(BUTTON_PIN) == LOW) { delay(10); }
        }
    }

    // 采集模式
    if (capturing) {
        if (millis() - last_sample_time >= SAMPLE_RATE_MS) {
            last_sample_time = millis();
            sensors_event_t a, g, temp;
            mpu.getEvent(&a, &g, &temp);
            if (sample_count < SAMPLE_COUNT) {
                int idx = sample_count * 3;
                features[idx]     = a.acceleration.x;
                features[idx + 1] = a.acceleration.y;
                features[idx + 2] = a.acceleration.z;
                sample_count++;
                Serial.print("采集进度: ");
                Serial.print(sample_count);
                Serial.print("/");
                Serial.println(SAMPLE_COUNT);
            }
            if (millis() - capture_start_time >= CAPTURE_DURATION_MS) {
                capturing = false;
                Serial.println("Capture complete, running inference...");
                run_inference();
            }
        }
    }

    // 结果显示超时检测（3 秒后熄灭）
    if (showing_result) {
        if (millis() - result_start_time >= 3000) {
            showing_result = false;
            digitalWrite(LED_RED_PIN, LOW);
            digitalWrite(LED_GREEN_PIN, LOW);
            digitalWrite(LED_BLUE_PIN, LOW);
            Serial.println("结果显示结束，灯已熄灭");
        }
    }
}

void run_inference() {
    signal_t sig;
    sig.get_data     = raw_feature_get_data;
    sig.total_length = FEATURE_SIZE;

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR res = run_classifier(&sig, &result, false);
    if (res != EI_IMPULSE_OK) {
        Serial.print("ERR: Failed to run classifier (");
        Serial.print(res);
        Serial.println(")");
        return;
    }
    // 找最高置信并点灯
    float max_value = 0;
    int max_index = -1;
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > max_value) {
            max_value = result.classification[i].value;
            max_index = i;
        }
    }
    if (max_index != -1) {
        print_prediction_and_light(
            ei_classifier_inferencing_categories[max_index],
            max_value * 100
        );
    }
}

void print_prediction_and_light(const char* label, float confidence) {
    // 熄灭之前的灯
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);

    Serial.print("Prediction: ");
    Serial.print(label);
    Serial.print(" (");
    Serial.print(confidence);
    Serial.println("%)");

    // 按 label 点亮对应灯
    if (strcmp(label, "Z") == 0) {
        digitalWrite(LED_RED_PIN, HIGH);
    } else if (strcmp(label, "O") == 0) {
        digitalWrite(LED_BLUE_PIN, HIGH);
    } else if (strcmp(label, "V") == 0) {
        digitalWrite(LED_GREEN_PIN, HIGH);
    }

    // 进入结果显示状态，记录开始时间
    showing_result = true;
    result_start_time = millis();
}
