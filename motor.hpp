#pragma once

#include <cstdint>

#include "brushed_motor_driver.hpp"

namespace XRMotor
{

template <typename EncoderT>
class Motor
{
 public:
  struct Parameters
  {
    Parameters()
        : encoder_counts_per_motor_rev(1.0f),
          gear_ratio(1.0f),
          output_deadzone(0.0f),
          min_effective_output(0.0f),
          invert_output(false)
    {
    }

    float encoder_counts_per_motor_rev;
    float gear_ratio;
    float output_deadzone;
    float min_effective_output;
    bool invert_output;
  };

  Motor(BrushedMotorDriver& driver, EncoderT& encoder,
        Parameters params = Parameters());

  LibXR::ErrorCode Init(uint32_t pwm_frequency_hz);
  LibXR::ErrorCode SetOutput(float output);
  LibXR::ErrorCode Coast();
  LibXR::ErrorCode Brake();
  LibXR::ErrorCode Stop();
  int32_t GetEncoderCount() const;
  int32_t GetAndResetEncoderDelta();
  float GetMotorRevolutions() const;
  float GetWheelRevolutions() const;
  float GetMotorAngleRad() const;
  float GetWheelAngleRad() const;
  void ResetEncoder();

 private:
  float NormalizeOutput(float output) const;
  float CountsToMotorRevolutions(int32_t counts) const;

  BrushedMotorDriver& driver_;
  EncoderT& encoder_;
  Parameters params_;
};

namespace
{

constexpr float kTwoPi = 6.28318530717958647692f;

inline float ClampUnit(float value)
{
  if (value > 1.0f)
  {
    return 1.0f;
  }
  if (value < -1.0f)
  {
    return -1.0f;
  }
  return value;
}

}  // namespace

template <typename EncoderT>
Motor<EncoderT>::Motor(BrushedMotorDriver& driver, EncoderT& encoder,
                       Parameters params)
    : driver_(driver), encoder_(encoder), params_(params)
{
}

template <typename EncoderT>
LibXR::ErrorCode Motor<EncoderT>::Init(uint32_t pwm_frequency_hz)
{
  return driver_.Init(pwm_frequency_hz);
}

template <typename EncoderT>
LibXR::ErrorCode Motor<EncoderT>::SetOutput(float output)
{
  return driver_.SetOutput(NormalizeOutput(output));
}

template <typename EncoderT>
LibXR::ErrorCode Motor<EncoderT>::Coast()
{
  return driver_.Coast();
}

template <typename EncoderT>
LibXR::ErrorCode Motor<EncoderT>::Brake()
{
  return driver_.Brake();
}

template <typename EncoderT>
LibXR::ErrorCode Motor<EncoderT>::Stop()
{
  return Coast();
}

template <typename EncoderT>
int32_t Motor<EncoderT>::GetEncoderCount() const
{
  return encoder_.GetCount();
}

template <typename EncoderT>
int32_t Motor<EncoderT>::GetAndResetEncoderDelta()
{
  return encoder_.GetAndResetDelta();
}

template <typename EncoderT>
float Motor<EncoderT>::GetMotorRevolutions() const
{
  return CountsToMotorRevolutions(encoder_.GetCount());
}

template <typename EncoderT>
float Motor<EncoderT>::GetWheelRevolutions() const
{
  if (params_.gear_ratio <= 0.0f)
  {
    return 0.0f;
  }
  return GetMotorRevolutions() / params_.gear_ratio;
}

template <typename EncoderT>
float Motor<EncoderT>::GetMotorAngleRad() const
{
  return GetMotorRevolutions() * kTwoPi;
}

template <typename EncoderT>
float Motor<EncoderT>::GetWheelAngleRad() const
{
  return GetWheelRevolutions() * kTwoPi;
}

template <typename EncoderT>
void Motor<EncoderT>::ResetEncoder()
{
  encoder_.Reset();
}

template <typename EncoderT>
float Motor<EncoderT>::NormalizeOutput(float output) const
{
  output = ClampUnit(output);

  if (params_.invert_output)
  {
    output = -output;
  }

  float magnitude = (output >= 0.0f) ? output : -output;
  if (magnitude <= params_.output_deadzone)
  {
    return 0.0f;
  }

  if (magnitude > 0.0f && params_.min_effective_output > 0.0f &&
      magnitude < params_.min_effective_output)
  {
    magnitude = params_.min_effective_output;
  }

  return (output >= 0.0f) ? magnitude : -magnitude;
}

template <typename EncoderT>
float Motor<EncoderT>::CountsToMotorRevolutions(int32_t counts) const
{
  if (params_.encoder_counts_per_motor_rev <= 0.0f)
  {
    return 0.0f;
  }
  return static_cast<float>(counts) / params_.encoder_counts_per_motor_rev;
}

}  // namespace XRMotor
