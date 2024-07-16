#include <Utils.h>

// Struct C_L29N_Motor begin

// Public

void C_L298N_Motor::SwitchState() {
  _isOn ^= 1;
  ProcessState();
}

void C_L298N_Motor::ProcessReverse() {
  uint8_t tempSpeed = _speed;
  SlowDown(0);
  _motorRotation = static_cast<Rotation>((int)_motorRotation ^ 1);
  Rotate();
  delay(_speedChangeTime);
  SpeedUp(tempSpeed);
}

void C_L298N_Motor::SetupPins() {
  const int freq = 30000;
  const int pwmChannel = 0;
  const int resolution = 8;

  pinMode(PIN_MOTOR_SPEED, OUTPUT);
  pinMode(PIN_MOTOR_1, OUTPUT);
  pinMode(PIN_MOTOR_2, OUTPUT);

  ledcAttachChannel(PIN_MOTOR_SPEED, freq, resolution, pwmChannel);
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
}

void C_L298N_Motor::SetSpeed(uint8_t speed) {
  if (_speed > speed)
    SlowDown(speed);
  else
    SpeedUp(speed);
}

bool C_L298N_Motor::GetState() { return _isOn; }

uint8_t C_L298N_Motor::GetSpeed() { return _speed; }

// Private

void C_L298N_Motor::ProcessState() {
  if (_isOn) {
    Rotate();
    return;
  }
  SlowDown(0);
  ledcWrite(PIN_MOTOR_SPEED, _speed);
  digitalWrite(PIN_MOTOR_1, LOW);
  digitalWrite(PIN_MOTOR_2, LOW);
}

void C_L298N_Motor::Rotate() {
  switch (_motorRotation) {
  case (Rotation::CLOCKWISE):
    digitalWrite(PIN_MOTOR_1, LOW);
    digitalWrite(PIN_MOTOR_2, HIGH);
    break;
  case (Rotation::ANTI_CLOCKWISE):
    digitalWrite(PIN_MOTOR_1, HIGH);
    digitalWrite(PIN_MOTOR_2, LOW);
    break;
  default:
    digitalWrite(PIN_MOTOR_1, LOW);
    digitalWrite(PIN_MOTOR_2, LOW);
    break;
  }
}

void C_L298N_Motor::SpeedUp(uint8_t aimSpeed) {
  aimSpeed = (aimSpeed > 255) ? 255 : aimSpeed;
  while (_speed < aimSpeed - _speedChangeRate) {
    _speed += _speedChangeRate;
    ledcWrite(PIN_MOTOR_SPEED, _speed);
    delay(_speedChangeTime);
  }
  _speed = aimSpeed;
  ledcWrite(PIN_MOTOR_SPEED, _speed);
}

void C_L298N_Motor::SlowDown(uint8_t aimSpeed) {
  aimSpeed = (aimSpeed < 0) ? 0 : aimSpeed;
  while (_speed > aimSpeed + _speedChangeRate) {
    _speed -= _speedChangeRate;
    ledcWrite(PIN_MOTOR_SPEED, _speed);
    delay(_speedChangeTime);
  }
  _speed = aimSpeed;
  ledcWrite(PIN_MOTOR_SPEED, _speed);
}



// Struct C_L298N_Motor end


// Struct C_Timer begin

// Public

void C_Timer::SetTime(uint8_t hour, uint8_t min, uint8_t sec) {
  hour = ((hour > 23) ? 23 : hour);
  min = ((min > 59) ? 59 : min);
  sec = ((sec > 59) ? 59 : sec);
  _time = hour * 3600 + min * 60 + sec;
}

const String C_Timer::GetTimeRemaining() {
  std::chrono::seconds temp_all_time = std::chrono::seconds(_time);
  std::chrono::seconds temp_elapsed_time =
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _startTime);
  uint32_t time_left = (temp_all_time >= temp_elapsed_time) ? (temp_all_time - temp_elapsed_time).count() : 0;
  return String(time_left / 3600) + ":" + String(time_left % 3600 / 60) + ":" + String(time_left % 60);
}

void C_Timer::CheckTime(C_L298N_Motor& motor) {
  if (motor.GetState() &&
    _isOn &&
    std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _startTime) >=
    std::chrono::seconds(_time)) {
    motor.SwitchState();
    SetState(false);
    SetTime(0, 0, 0);
  }
}

void C_Timer::RefreshStartTime() { _startTime = std::chrono::steady_clock::now(); }

void C_Timer::SetState(bool state) { _isOn = state; }

bool C_Timer::GetState() { return _isOn; }

// Struct C_Timer end


// Struct C_INA219_Sensor begin

// Public

void C_INA219_Sensor::SetupINA219() {
  if (!_ina219.begin()) {
    while (1) { delay(10); }
  }
}

void C_INA219_Sensor::UpdateSensor() {
  _voltage_V = _ina219.getBusVoltage_V();
  _current_mA = _ina219.getCurrent_mA();
}

std::pair<float, float> C_INA219_Sensor::GetSensorData() {
  return { _voltage_V, _current_mA };
}

// Struct C_INA219_Sensor end