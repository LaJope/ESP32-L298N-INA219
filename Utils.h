#pragma once

// Для подключения датчика силы тока и напряжения INA219.
// Adagruin INA219 by Adafruit
#include <Wire.h>
#include <Adafruit_INA219.h>

#include <chrono>

// Пины для управления мотором спомощью L298N.
// Пин 14 отвечает за управление скоростью мотора.
#define PIN_MOTOR_SPEED 14
// Пины 26-27 отвечают за направление вращения мотора.
#define PIN_MOTOR_1 26
#define PIN_MOTOR_2 27


struct C_L298N_Motor {
private:

  enum class Rotation : byte {
    CLOCKWISE = 0,
    ANTI_CLOCKWISE = 1
  };

  const uint8_t _speedChangeRate = 20;
  const uint16_t _speedChangeTime = 200;
  uint8_t _speed = 0;
  bool _isOn = false;
  Rotation _motorRotation = Rotation::CLOCKWISE;

public:

  void SwitchState();
  void ProcessReverse();

  void SetupPins();
  void SetSpeed(uint8_t speed);

  bool GetState();
  uint8_t GetSpeed();

private:

  void ProcessState();
  void Rotate();
  void SpeedUp(uint8_t aimSpeed);
  void SlowDown(uint8_t aimSpeed);
};

struct C_Timer {
private:

  uint32_t _time = 0;
  std::chrono::steady_clock::time_point _startTime = std::chrono::steady_clock::now();
  bool _isOn = false;

public:

  void SetTime(uint8_t hour, uint8_t min, uint8_t sec);
  const String GetTimeRemaining();
  void CheckTime(C_L298N_Motor& motor);

  void RefreshStartTime();
  void SetState(bool state);
  bool GetState();
};

struct C_INA219_Sensor {
private:

  Adafruit_INA219 _ina219;
  float _voltage_V = 0.0f;
  float _current_mA = 0.0f;

public:

  void SetupINA219();
  void UpdateSensor();
  std::pair<float, float> GetSensorData();
};